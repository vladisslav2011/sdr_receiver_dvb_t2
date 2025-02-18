#ifndef RX_BASE_CPP
#define RX_BASE_CPP

#include "rx_base.h"

//-------------------------------------------------------------------------------------------
template<typename T>int rx_base<T>::init(uint32_t _rf_frequency_hz, int _gain)
{
    ch_frequency = _rf_frequency_hz;
    rf_frequency = ch_frequency;
    gain = _gain;
    if(gain < 0) {
        agc = true;
        gain = 0;
    }

    int ret = hw_init(_rf_frequency_hz, _gain);
    if(ret < 0)
        return ret;

    set_gain_db(_gain);

    uint max_len_out = len_out_device * max_blocks;

    buffer_a.resize(max_len_out);
    buffer_b.resize(max_len_out);
    ptr_buffer = &buffer_a[0];
    swap_buffer = true;

    signal.agc = agc;

    demodulator = new dvbt2_demodulator(id_airspy, sample_rate);
    thread = new QThread;
    thread->setObjectName("demod");
    demodulator->moveToThread(thread);
    connect(this, &rx_base::execute, demodulator, &dvbt2_demodulator::execute);
    connect(this, &rx_base::stop_demodulator, demodulator, &dvbt2_demodulator::stop);
    connect(demodulator, &dvbt2_demodulator::finished, demodulator, &dvbt2_demodulator::deleteLater);
    connect(demodulator, &dvbt2_demodulator::finished, thread, &QThread::quit, Qt::DirectConnection);
    connect(thread, &QThread::finished, thread, &QThread::deleteLater);
    thread->start(QThread::TimeCriticalPriority);

    return ret;
}
//-------------------------------------------------------------------------------------------
template<typename T>void rx_base<T>::set_biastee(const bool state)
{
    (void) state;
}
//-------------------------------------------------------------------------------------------
template<typename T>void rx_base<T>::reset()
{
    signal.reset = false;
    rf_frequency = ch_frequency;
    signal.coarse_freq_offset = 0.0;
    signal.change_frequency = true;
    signal.correct_resample = 0.0;
    if(agc) {
        gain = 0;
    }
    signal.gain_offset = 0;
    signal.change_gain = true;
    ptr_buffer = &buffer_a[0];
    swap_buffer = true;
    len_buffer = 0;
    blocks = 1;
    set_rf_frequency();
    set_gain();
    conv.reset();
}
//-------------------------------------------------------------------------------------------
template<typename T>void rx_base<T>::set_rf_frequency()
{
    if(!signal.frequency_changed) {
        end_wait_frequency_changed = clock();
        float mseconds = (end_wait_frequency_changed - start_wait_frequency_changed) /
                         (CLOCKS_PER_SEC / 1000);
        if(mseconds > 100) {
            signal.frequency_changed = true;
            on_frequency_changed();
            emit radio_frequency(rf_frequency);
        }
    }
    if(signal.change_frequency) {
        signal.change_frequency = false;
        int err = hw_set_frequency();
        if(err != 0) {
            emit status(err);
        }
        else{
            signal.frequency_changed = false;
            start_wait_frequency_changed = clock();
        }
    }
}
//-------------------------------------------------------------------------------------------
template<typename T>void rx_base<T>::set_gain()
{
    if(!signal.gain_changed) {
        end_wait_gain_changed = clock();
        float mseconds = (end_wait_gain_changed - start_wait_gain_changed) /
                         (CLOCKS_PER_SEC / 1000);
        if(mseconds > 10) {
            signal.gain_changed = true;
            on_gain_changed();
            emit level_gain(gain);
        }
    }
    if(agc && signal.change_gain) {

        const auto old_gain = gain;
        gain += signal.gain_offset;
        if(gain > GAIN_MAX) {
            gain = GAIN_MIN;
        }
        if(gain < GAIN_MIN) {
            gain = GAIN_MAX;
        }
        signal.gain_offset = 0;
        if(old_gain == gain)
            return;
        int err =  hw_set_gain();
        if(err != 0) {
            emit status(err);
        }
        else{
            signal.gain_changed = false;
            start_wait_gain_changed = clock();
        }
    }
}
//----------------------------------------------------------------------------------------------------------------------------
template<typename T>void rx_base<T>::set_gain_db(int _gain)
{
    gain = _gain;
    int err = hw_set_gain();
    if(err != 0) {
        emit status(err);
    }
    else{
        signal.gain_changed = false;
        start_wait_gain_changed = clock();
    }
}
//-------------------------------------------------------------------------------------------
template<typename T>void rx_base<T>::update_gain_frequency()
{
    // coarse frequency setting
    set_rf_frequency();
    // AGC
    set_gain();
}
//-------------------------------------------------------------------------------------------
template<typename T>void rx_base<T>::rx_execute(int nsamples, float level_detect)
{
    if(nsamples == 0)
        return;
    len_buffer += nsamples;
    ptr_buffer += nsamples;

    if(demodulator->mutex->try_lock()) {

        if(signal.reset){
            reset();
            demodulator->mutex->unlock();
            return;
        }
        emit buffered(len_buffer/nsamples, max_blocks, level_detect);
        update_gain_frequency();

        if(swap_buffer) {
            emit execute(len_buffer, &buffer_a[0], level_detect, &signal);
            ptr_buffer = buffer_b.data();
        }
        else {
            emit execute(len_buffer, &buffer_b[0], level_detect, &signal);
            ptr_buffer = buffer_a.data();
        }
        swap_buffer = !swap_buffer;
        len_buffer = 0;
        blocks = 1;

        demodulator->mutex->unlock();
    }
    else {
        ++blocks;
        if(blocks > max_blocks){
            fprintf(stderr, "reset buffer blocks: %d\n", blocks);
            blocks = 1;
            len_buffer = 0;
            if(swap_buffer) {
                ptr_buffer = &buffer_a[0];
            }
            else {
                ptr_buffer = &buffer_b[0];
            }
        }
    }
}
//-------------------------------------------------------------------------------------------
template<typename T>void rx_base<T>::start()
{
    reset();
    int err;
    err = hw_start();
    if(err < 0) emit status(err);
    if(blocking_start)
    {
        emit stop_demodulator();
        if(thread->isRunning()) {
            thread->wait(1000);
        }
        emit finished();
    }
}
//-------------------------------------------------------------------------------------------
template<typename T>void rx_base<T>::stop()
{
    hw_stop();
    if(!blocking_start)
    {
        emit stop_demodulator();
        if(thread->isRunning()) {
            thread->wait(1000);
        }
        emit finished();
    }
}
//-------------------------------------------------------------------------------------------
template<typename T>int rx_base<T>::gain_min()
{
    return GAIN_MIN;
}
//-------------------------------------------------------------------------------------------
template<typename T>int rx_base<T>::gain_max()
{
    return GAIN_MAX;
}
//-------------------------------------------------------------------------------------------

#endif // RX_BASE_CPP
