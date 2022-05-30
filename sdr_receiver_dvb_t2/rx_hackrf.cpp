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
#include "rx_hackrf.h"

#include <QThread>
#include <QWaitCondition>
#include <QMutex>
#include <iostream>
#include <math.h>

//----------------------------------------------------------------------------------------------------------------------------
rx_hackrf::rx_hackrf(QObject *parent) : QObject(parent)
{
  hackrf_init(); /* call only once before the first open */

}
//----------------------------------------------------------------------------------------------------------------------------
rx_hackrf::~rx_hackrf()
{
  hackrf_exit(); /* call only once after last close */
}
//----------------------------------------------------------------------------------------------------------------------------
string rx_hackrf::error (int err)
{
    switch (err) {
       case HACKRF_SUCCESS:
          return "Success";
       case HACKRF_ERROR_INVALID_PARAM:
          return "Invalid parameter";
       case HACKRF_ERROR_NOT_FOUND:
          return "HackRF not found";
       case HACKRF_ERROR_BUSY:
          return "Device busy";
       case HACKRF_ERROR_NO_MEM:
          return "Out of memory error";
       case HACKRF_ERROR_LIBUSB:
          return "Libusb error";
       case HACKRF_ERROR_THREAD:
          return "HackRF thread error";
       case HACKRF_ERROR_STREAMING_THREAD_ERR:
          return "HackRF streaming thread error";
       case HACKRF_ERROR_STREAMING_STOPPED:
          return "HackRF error: streaming stopped";
       case HACKRF_ERROR_STREAMING_EXIT_CALLED:
          return "HackRF error: exit called";
       case HACKRF_ERROR_USB_API_VERSION:
          return "HackRF wrong USB api version";
       case HACKRF_ERROR_OTHER:
          return "HackRF error: other";
       default:
          return "Unknown error";
    }
}
//----------------------------------------------------------------------------------------------------------------------------
int rx_hackrf::get(string &_ser_no, string &_hw_ver)
{

  static std::vector<std::string> devices;
  static std::vector<unsigned char> hw_ver;
  std::string label;

  devices.resize(0);
  hw_ver.resize(0);
#ifdef LIBHACKRF_HAVE_DEVICE_LIST
  hackrf_device_list_t *list = hackrf_device_list();
  
  for (int i = 0; i < list->devicecount; i++) {
    if (list->serial_numbers[i]) {
      std::string serial (list->serial_numbers[i] );
      if (serial.length() > 6)
        serial = serial.substr(serial.length() - 6, 6);
      devices.push_back(serial);
      hw_ver.push_back(0);
    } else {
      devices.push_back("hackrf"); /* will pick the first one, serial number is required for choosing a specific one */
      hw_ver.push_back(0);
    }
  }
  
  hackrf_device_list_free(list);
#else
  int ret;
  hackrf_device *dev = NULL;
  ret = hackrf_open(&dev);
  if ( HACKRF_SUCCESS == ret )
  {
    std::string args = "hackrf=0";

    devices.push_back( args );

    ret = hackrf_close(dev);
  }

#endif
  _ser_no = devices[0];
  return 0;
}
//----------------------------------------------------------------------------------------------------------------------------
int rx_hackrf::init(double _rf_frequency, int _gain_db)
{
    int ret = 0;
    rf_frequency = _rf_frequency;
    gain_db = _gain_db;
    if(gain_db < 0) {
        gain_db = 78;
        agc = true;
    }
    sample_rate = 10000000.0f; // max for 10bit (10000000.0f for 8bit)
    ret = hackrf_open( &_dev );
    hackrf_set_sample_rate( _dev, sample_rate );
    hackrf_set_freq( _dev, uint64_t(rf_frequency) );
    gain_db = _gain_db;
    uint32_t bw = hackrf_compute_baseband_filter_bw( uint32_t(7000000.0) );
    ret = hackrf_set_baseband_filter_bandwidth( _dev, bw );
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

    frame = new dvbt2_frame(signal_out, mutex_out, id_hackrf, max_len_out, len_out_device, sample_rate);
    thread = new QThread;
    frame->moveToThread(thread);
    connect(this, &rx_hackrf::execute, frame, &dvbt2_frame::execute);
    connect(this, &rx_hackrf::stop_demodulator, frame, &dvbt2_frame::stop);
    connect(frame, &dvbt2_frame::finished, frame, &dvbt2_frame::deleteLater);
    connect(frame, &dvbt2_frame::finished, thread, &QThread::quit, Qt::DirectConnection);
    connect(thread, &QThread::finished, thread, &QThread::deleteLater);
    thread->start();

    return ret;
}

//----------------------------------------------------------------------------------------------------------------------------
void rx_hackrf::set_gain(int gain)
{
    int clip_gain =0;
    if(gain>=40)
    {
        clip_gain = 40;
        gain -=40;
    }else {
        clip_gain = gain;
        gain = 0;
    }
    hackrf_set_lna_gain( _dev, uint32_t(clip_gain) );
    std::cerr<<"LNA="<<clip_gain<<"\n";
    clip_gain=0;
    if(gain)
    {
        if(gain>=10)
        {
            clip_gain = 10;
            gain -=10;
        }else
            clip_gain = 0;
    }
    hackrf_set_amp_enable( _dev, clip_gain?1:0 );
    std::cerr<<"AMP="<<clip_gain<<"\n";
    clip_gain=0;
    if(gain)
    {
        if(gain>=50)
        {
            clip_gain = 50;
        }else
            clip_gain = gain+10;
    }
    hackrf_set_vga_gain( _dev, uint32_t(clip_gain) );
    std::cerr<<"VGA="<<clip_gain<<"\n";
}
//----------------------------------------------------------------------------------------------------------------------------
int rx_hackrf::callback(hackrf_transfer* transfer)
{
    if(!transfer) return 0;

    uint8_t *ptr = transfer->buffer;
    rx_hackrf *ctx = static_cast<rx_hackrf*>(transfer->rx_ctx);

    ctx->rx_execute(ptr,transfer->valid_length/2);

    if(!ctx->done)
    {
        return -1;
    }

    return 0;
}
//----------------------------------------------------------------------------------------------------------------------------
void rx_hackrf::rx_execute(void *in_ptr, int nsamples)
{
    int err;
    float mag=0;
    int8_t * ptr = (int8_t*)in_ptr;
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
        frame->get_signal_estimate(change_frequency, frequency_offset,
                                   change_gain, gain_offset);
        if(!frequency_changed) {
            end_wait_frequency_changed = clock();
            float mseconds = end_wait_frequency_changed - start_wait_frequency_changed;
            if(mseconds > 500000) frequency_changed = true;
        }
        if(change_frequency) {
            float correct = -frequency_offset / static_cast<float>(rf_frequency);
            frame->correct_resample(correct);
            rf_frequency += static_cast<int64_t>(frequency_offset);
            err = hackrf_set_freq( _dev, uint64_t(rf_frequency));
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
            set_gain(gain_db);
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
void rx_hackrf::start()
{
    int err;
    ptr_i_buffer = i_buffer_a;
    ptr_q_buffer = q_buffer_a;
    emit radio_frequency(rf_frequency);
    emit level_gain(gain_db);
    err = hackrf_start_rx(_dev, callback, (void*) this);
    len_buffer = 0;
    fprintf(stderr, "hackrf start rx %d\n", err);
}
//----------------------------------------------------------------------------------------------------------------------------
void rx_hackrf::stop()
{
    done = false;
    hackrf_stop_rx(_dev);
    hackrf_close(_dev);
    emit stop_demodulator();
    if(thread->isRunning()) thread->wait(1000);
    emit finished();
}
//----------------------------------------------------------------------------------------------------------------------------
