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
#ifndef RX_MIRISDR_H
#define RX_MIRISDR_H

#include <QObject>
#include <QTime>
#include <QApplication>
#include <string>
#include <mirisdr.h>

#include "DVB_T2/dvbt2_frame.h"

typedef std::string string;

class rx_miri : public QObject
{
    Q_OBJECT
public:
    explicit rx_miri(QObject* parent = nullptr);
    ~rx_miri();

    string error (int err);
    int get(string &_ser_no, string &_hw_ver);
    int init(double _rf_frequency, int _gain_db);
    dvbt2_frame* frame;

signals:
    void execute(int _len_in, short* _i_in, short* _q_in, bool _frequence_changed, bool _gain_changed);
    void status(int _err);
    void radio_frequency(double _rf);
    void level_gain(int _gain);
    void stop_demodulator();
    void finished();
    void buffered(int nbuffers, int totalbuffers);

public slots:
    void start();
    void stop();
    void set_frequency(double _freq);
    void set_gain(int gain);
private:
    void rx_execute(void *ptr, int nsamples);
    static void callback(unsigned char *buf, uint32_t len, void *context);

private:
    QThread* thread;
    QWaitCondition* signal_out;
    QMutex* mutex_out;
    mirisdr_dev_t *_dev;
    int err;
    float sample_rate;
    const int len_out_device = 256*1024;
    const int max_blocks = 128;//12288;//768
    int blocks = 1;
    int max_len_out = 0;
    int len_buffer = 0;
    short* i_buffer_a;
    short* q_buffer_a;
    short* i_buffer_b;
    short* q_buffer_b;
    short* ptr_i_buffer;
    short* ptr_q_buffer;
    bool swap_buffer = true;
    int64_t rf_bandwidth_hz;
    int64_t sample_rate_hz;
    clock_t start_wait_frequency_changed;
    clock_t end_wait_frequency_changed;
    int64_t rf_frequency;
    float frequency_offset = 0.0f;
    bool change_frequency = false;
    bool frequency_changed = true;
    clock_t start_wait_gain_changed;
    clock_t end_wait_gain_changed;
    int32_t gain_db;
    bool agc = false;
    int gain_offset = 0;
    bool change_gain = false;
    bool gain_changed = true;
    bool done = true;
};

#endif // RX_MIRISDR_H
