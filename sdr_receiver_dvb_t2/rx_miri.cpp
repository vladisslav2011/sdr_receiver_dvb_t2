/*
 *  Copyright 2020 Oleg Malyutin.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/
#include "rx_miri.h"

#include <QThread>
#include <QWaitCondition>
#include <QMutex>
#include <unistd.h>

//----------------------------------------------------------------------------------------------------------------------------
rx_miri::rx_miri(QObject *parent) : QObject(parent),
i_buffer_a(nullptr),
q_buffer_a(nullptr),
i_buffer_b(nullptr),
q_buffer_b(nullptr)
{
    fprintf(stderr,"rx_miri::rx_miri\n");

}
//----------------------------------------------------------------------------------------------------------------------------
rx_miri::~rx_miri()
{
    if(i_buffer_a)
    {
        delete i_buffer_a;
        delete q_buffer_a;
        delete i_buffer_b;
        delete q_buffer_b;
    }
    fprintf(stderr,"rx_miri::~rx_miri\n");
}
//----------------------------------------------------------------------------------------------------------------------------
string rx_miri::error (int err)
{
    switch (err) {
       case 0:
          return "Success";
       case -1:
          return "Mirisdr error";
       default:
          return "Unknown error";
    }
}
//----------------------------------------------------------------------------------------------------------------------------
int rx_miri::get(string &_ser_no, string &_hw_ver)
{

  static std::vector<std::string> devices;
  static std::vector<unsigned char> hw_ver;
  std::string label;

  devices.resize(0);
  hw_ver.resize(0);
  for (unsigned int i = 0; i < mirisdr_get_device_count(); i++) {
    devices.push_back( std::string(mirisdr_get_device_name( i ) ));
  }
  if(devices.size() == 0)
    return -1;
  _ser_no = devices[0];
  return 0;
}
//----------------------------------------------------------------------------------------------------------------------------
int rx_miri::init(double _rf_frequency, int _gain_db)
{
    int ret = 0;
    static char fmt[] = "336_S16\x00";
    //static char fmt[] = "504_S16\x00";
    fprintf(stderr,"miri init\n");
    rf_frequency = _rf_frequency;
    gain_db = _gain_db;
    if(gain_db < 0) {
        gain_db = 0;
        agc = true;
    }
    sample_rate = 10000000.0f; // max for 10bit (10000000.0f for 8bit)
    ret = mirisdr_open( &_dev, 0 );
    mirisdr_set_sample_format( _dev, fmt);
    mirisdr_set_sample_rate( _dev, sample_rate );
    mirisdr_set_center_freq( _dev, uint32_t(rf_frequency) );
    gain_db = _gain_db;
    ret = mirisdr_set_bandwidth( _dev, 12000000 );
    //mirisdr_set_bias( _dev, 1);
    set_gain(gain_db);

    if(ret != 0) return ret;

    max_len_out = len_out_device * max_blocks;
    len_buffer = static_cast<unsigned int>(max_len_out);
    i_buffer_a = new short[len_buffer];
    q_buffer_a = new short[len_buffer];
    i_buffer_b = new short[len_buffer];
    q_buffer_b = new short[len_buffer];
    ptr_i_buffer = i_buffer_a;
    ptr_q_buffer = q_buffer_a;

    mutex_out = new QMutex;
    signal_out = new QWaitCondition;

    frame = new dvbt2_frame(signal_out, mutex_out, id_miri, max_len_out, len_out_device, sample_rate);
    thread = new QThread;
    frame->moveToThread(thread);
    connect(this, &rx_miri::execute, frame, &dvbt2_frame::execute);
    connect(this, &rx_miri::stop_demodulator, frame, &dvbt2_frame::stop);
    connect(frame, &dvbt2_frame::finished, frame, &dvbt2_frame::deleteLater);
    connect(frame, &dvbt2_frame::finished, thread, &QThread::quit, Qt::DirectConnection);
    connect(thread, &QThread::finished, thread, &QThread::deleteLater);
    thread->start();

    return ret;
}
//----------------------------------------------------------------------------------------------------------------------------
void rx_miri::set_frequency(double _freq)
{
    int err=mirisdr_set_center_freq( _dev, uint32_t(_freq) );
    if(err != 0)
        emit status(err);
}
//----------------------------------------------------------------------------------------------------------------------------
void rx_miri::set_gain(int gain)
{
    mirisdr_set_tuner_gain(_dev, gain);
}
//----------------------------------------------------------------------------------------------------------------------------
void rx_miri::callback(unsigned char *buf, uint32_t len, void *context)
{
    if(!buf) return;

    rx_miri *ctx = static_cast<rx_miri*>(context);

    ctx->rx_execute(buf,len/4);
}
//----------------------------------------------------------------------------------------------------------------------------
void rx_miri::rx_execute(void *in_ptr, int nsamples)
{
    int err=0;
    float mag=0;
    int16_t * ptr = (int16_t*)in_ptr;
    for(int i = 0; i < nsamples; ++i) {
        float ti = ptr_i_buffer[i] = ptr[i*2];
        float tq = ptr_q_buffer[i] = ptr[i*2+1];
        mag+=sqrtf(ti*ti+tq*tq);
    }
    mag/=float(nsamples);
//    std::cerr<<"B="<<nsamples<<" mag="<<mag<<"\n";
    len_buffer += nsamples;
    ptr_i_buffer += nsamples;
    ptr_q_buffer += nsamples;

    if(mutex_out->try_lock()) {
        emit buffered(blocks, max_blocks);
        frame->get_signal_estimate(change_frequency, frequency_offset,
                                   change_gain, gain_offset);
        if(!frequency_changed) {
            end_wait_frequency_changed = clock();
            float mseconds = end_wait_frequency_changed - start_wait_frequency_changed;
            if(mseconds > 500000) frequency_changed = true;
        }
        if(change_frequency) {
            float correct = -frequency_offset / static_cast<float>(rf_frequency);
            //frame->correct_resample(correct);
            rf_frequency += static_cast<int64_t>(frequency_offset);
            //err = mirisdr_set_center_freq( _dev, uint32_t(rf_frequency) );
            if(err < 0) emit status(err);
            frequency_changed = false;
            emit radio_frequency(rf_frequency);
            start_wait_frequency_changed = clock();
        }
        // AGC
        if(!gain_changed) {
            end_wait_gain_changed = clock();
            float mseconds = end_wait_gain_changed - start_wait_gain_changed;
            if(mseconds > 100) gain_changed = true;
        }
        if(agc && change_gain) {
            gain_changed = false;
            gain_db += gain_offset;
            if(err < 0) emit status(err);
            start_wait_gain_changed = clock();
            emit level_gain(gain_db);
        }
        if(swap_buffer) {
            swap_buffer = false;
            emit execute(len_buffer, i_buffer_a, q_buffer_a, frequency_changed, gain_changed);
            mutex_out->unlock();
            ptr_i_buffer = i_buffer_b;
            ptr_q_buffer = q_buffer_b;
        }
        else {
            swap_buffer = true;
            emit execute(len_buffer, i_buffer_b, q_buffer_b, frequency_changed, gain_changed);
            mutex_out->unlock();
            ptr_i_buffer = i_buffer_a;
            ptr_q_buffer = q_buffer_a;
        }
        len_buffer = 0;
        blocks = 1;
    }
    else {
        ++blocks;
        if(blocks > max_blocks) {
            fprintf(stderr, "reset buffer blocks: %d\n", blocks);
            blocks = 1;
            len_buffer = 0;
            if(swap_buffer) {
                ptr_i_buffer = i_buffer_a;
                ptr_q_buffer = q_buffer_a;
            }
            else {
                ptr_i_buffer = i_buffer_b;
                ptr_q_buffer = q_buffer_b;
            }
        }
    }
}
//----------------------------------------------------------------------------------------------------------------------------
void rx_miri::start()
{
    int err=0;
    ptr_i_buffer = i_buffer_a;
    ptr_q_buffer = q_buffer_a;
    emit radio_frequency(rf_frequency);
    emit level_gain(gain_db);
    err = mirisdr_reset_buffer(_dev);
    err = mirisdr_read_async( _dev, callback, (void *)this, 64, len_out_device );
    fprintf(stderr, "miri start rx %d\n", err);
    len_buffer = 0;
    mirisdr_close(_dev);
    emit stop_demodulator();
    if(thread->isRunning()) thread->wait(1000);
    emit finished();
}
//----------------------------------------------------------------------------------------------------------------------------
void rx_miri::stop()
{
    done = false;
    mirisdr_cancel_async( _dev );
}
//----------------------------------------------------------------------------------------------------------------------------
