#ifndef RX_INTERFACE_H
#define RX_INTERFACE_H

#include <QObject>
#include <QTimer>
#include <string>
#include <vector>

#include "DVB_T2/dvbt2_demodulator.h"

class rx_interface : public QObject
{
    Q_OBJECT
public:
    explicit rx_interface(QObject *parent = nullptr):QObject(parent)
    {
    }
    virtual ~rx_interface()
    {
    }
    virtual int get(std::string &_ser_no, std::string &_hw_ver) = 0;
    virtual std::string error (int err) = 0;
    virtual int init(uint32_t _rf_frequency_hz, int _gain) = 0;
    virtual void update_gain_frequency_direct()
    {
    }
    virtual const QString thread_name() = 0;
    virtual const QString dev_name() = 0;
    virtual int gain_min() = 0;
    virtual int gain_max() = 0;
    virtual void set_biastee(const bool state) = 0;

    dvbt2_demodulator *demodulator = nullptr;
signals:
    void execute(int _len_in, complex* _in, float _level_estimate, signal_estimate* signal_);
    void status(int _err);
    void radio_frequency(double _rf);
    void level_gain(int _gain);
    void stop_demodulator();
    void finished();
    void buffered(int nbuffers, int totalbuffers, float level);

public slots:
    virtual void start() = 0;
    virtual void stop() = 0;
    virtual void set_gain_db(int gain) = 0;

};

#endif // RX_INTERFACE_H
