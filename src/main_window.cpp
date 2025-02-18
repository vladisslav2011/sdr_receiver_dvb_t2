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
#include "main_window.h"
#include "ui_main_window.h"
#include <QAction>

//------------------------------------------------------------------------------------------------
main_window::main_window(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::main_window)
{
    ui->setupUi(this);

    qRegisterMetaType<fec_frame>();
    qRegisterMetaType<idx_plp_simd_t>();
    qRegisterMetaType<bch_decoder::in_t>();
#ifdef USE_SDRPLAY
    connect(ui->action_sdrplay, SIGNAL(triggered()), this, SLOT(open_sdrplay()));
    QAction * action_sdrplay = new QAction("SDRPlay", this);
    ui->menu_open->addAction(action_sdrplay);
    connect(action_sdrplay, SIGNAL(triggered()), this, SLOT(open_sdrplay()));
#endif
    connect(ui->action_airspy, SIGNAL(triggered()), this, SLOT(open_airspy()));
    ui->action_plutosdr->setVisible(false);
    ui->action_plutosdr->setEnabled(false);
#ifndef WIN32
    ui->action_plutosdr->setVisible(true);
    ui->action_plutosdr->setEnabled(true);
    connect(ui->action_plutosdr, SIGNAL(triggered()), this, SLOT(open_plutosdr()));
#endif
#ifdef USE_HACKRF
    QAction * action_hackrf = new QAction("HackRF", this);
    ui->menu_open->addAction(action_hackrf);
    connect(action_hackrf, SIGNAL(triggered()), this, SLOT(open_hackrf()));
#endif
#ifdef USE_MIRI
    QAction * action_miri = new QAction("Miri SDR", this);
    ui->menu_open->addAction(action_miri);
    connect(action_miri, SIGNAL(triggered()), this, SLOT(open_miri()));
#endif
#ifdef USE_USRP
    QAction * action_usrp = new QAction("USRP SDR", this);
    ui->menu_open->addAction(action_usrp);
    connect(action_usrp, SIGNAL(triggered()), this, SLOT(open_usrp()));
#endif
    QAction * action_raw = new QAction("RAW IQ file", this);
    ui->menu_open->addAction(action_raw);
    connect(action_raw, SIGNAL(triggered()), this, SLOT(open_raw()));

    connect(ui->action_exit, SIGNAL(triggered()), this, SLOT(close()));

    ui->tab_widget->setCurrentIndex(0);
    for(int i = 1; i < ui->tab_widget->count(); ++i) ui->tab_widget->setTabEnabled(i, false);
    ui->push_button_start->setEnabled(false);
    ui->push_button_stop->setEnabled(false);
    ui->push_button_ts_open_file->setEnabled(false);
    ui->push_button_ts_apply->setEnabled(false);

    p1_spectrograph = new plot(ui->widget_fft_plot_p1, type_spectrograph, "Fast Fourier transform spectrum");
    p1_constelation = new plot(ui->widget_constellation_plot_p1, type_constelation, "Constellation diagram");
    p1_correlation_oscilloscope = new plot(ui->widget_p1_correlation, type_oscilloscope, "P1 preamble correlation");
    p2_spectrograph = new plot(ui->widget_fft_plot_p2, type_spectrograph, "Fast Fourier transform spectrum");
    p2_constelation = new plot(ui->widget_constellation_plot_p2, type_constelation, "Constellation diagram");
    data_spectrograph = new plot(ui->widget_fft_plot_data, type_spectrograph, "Fast Fourier transform spectrum");
    data_constelation = new plot(ui->widget_constellation_plot_data, type_constelation, "Constellation diagram");
    fc_spectrograph = new plot(ui->widget_fft_plot_fc, type_spectrograph, "Fast Fourier transform spectrum");
    fc_constelation = new plot(ui->widget_constellation_plot_fc, type_constelation, "Constellation diagram");
    qam_constelation = new plot(ui->widget_qam_plot, type_constelation, "Constellation diagram");
    p2_equalizer_oscilloscope = new plot(ui->widget_equalizer_p2, type_oscilloscope_2, "Equalizer estimation");
    frequency_offset = new plot(ui->widget_fq_offset, type_null_indicator, "Frequency estimation");
    ldpc_stats = new plot(ui->widget_ldpc_stats, type_ldpc_stats, "LDPC stats");

    button_group_p2_symbol = new QButtonGroup;
    button_group_p2_symbol->addButton(ui->radio_button_l1presignaling, 0);
    button_group_p2_symbol->addButton(ui->radio_button_l1postsignaling, 1);
    button_group_p2_symbol->addButton(ui->radio_button_data, 2);
    connect(button_group_p2_symbol, static_cast<void(QButtonGroup::*)(int)>(&QButtonGroup::buttonClicked),
            this, &main_window::set_show_p2_symbol);
//    connect(button_group_p2_symbol, &QButtonGroup::idClicked, this, &main_window::set_show_p2_symbol);
    std::vector<const char *> fir_names{};
    filter_decimator::get_filter_names(fir_names);
    for(auto & it: fir_names)
        ui->comboBoxFIR->addItem(it);
    ui->comboBoxFIR->setCurrentIndex(0);

}
//------------------------------------------------------------------------------------------------
main_window::~main_window()
{
    delete ui;
    delete p1_spectrograph;
    delete p1_constelation;
    delete p1_correlation_oscilloscope;
    delete p2_spectrograph;
    delete p2_constelation;
    delete data_spectrograph;
    delete data_constelation;
    delete fc_spectrograph;
    delete fc_constelation;
    delete qam_constelation;
    delete p2_equalizer_oscilloscope;
    delete frequency_offset;
    delete button_group_p2_symbol;
    delete ldpc_stats;
}
//------------------------------------------------------------------------------------------------
//void main_window::closeEvent(QCloseEvent *event)
//{
//    Q_UNUSED(event);
//    switch (id_device) {
//    case id_sdrplay:
//        break;
//    case id_airspy:
//        break;
//    case id_plutosdr:
////        if(thread == nullptr) ptr_plutosdr->reboot();
//        break;
//    }
//    if(thread != nullptr) {
//        emit stop_device();
//        if(thread->isRunning()) thread->wait(1000);
//    }
//}
//------------------------------------------------------------------------------------------------
void main_window::on_check_box_agc_stateChanged(int arg1)
{
    ui->spinBoxGain->setEnabled(!(arg1 == 2));
}
//---------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_SDRPLAY
void main_window::open_sdrplay()
{
    open_dev<rx_sdrplay>();
}
#endif
//---------------------------------------------------------------------------------------------------------------------------------
void main_window::open_airspy()
{
    open_dev<rx_airspy>();
}
//------------------------------------------------------------------------------------------------
#ifndef WIN32
void main_window::open_plutosdr()
{
    open_dev<rx_plutosdr>();
}
#endif
#ifdef USE_MIRI
//---------------------------------------------------------------------------------------------------------------------------------
void main_window::open_miri()
{
    open_dev<rx_miri>();
}
#endif
#ifdef USE_HACKRF
//---------------------------------------------------------------------------------------------------------------------------------
void main_window::open_hackrf()
{
    open_dev<rx_hackrf>();
}
#endif
#ifdef USE_USRP
//---------------------------------------------------------------------------------------------------------------------------------
void main_window::open_usrp()
{
    open_dev<rx_usrp>();
}
#endif
//---------------------------------------------------------------------------------------------------------------------------------
void main_window::open_raw()
{
    open_dev<rx_raw>();
}
//---------------------------------------------------------------------------------------------------------------------------------
template<typename T> void main_window::open_dev()
{
    int err;
    std::string ser_no;
    std::string hw_ver;
    if(ptr_dev)
        delete ptr_dev;
    ptr_dev = new T;
    err = ptr_dev->get(ser_no, hw_ver);
    ui->text_log->insertPlainText("Get " + ptr_dev->dev_name() +":" +
                                  QString::fromStdString(ptr_dev->error(err)) + "\n");
    if(err < 0) return;

    ui->label_name->setText("Name : " + ptr_dev->dev_name());
    ui->label_ser_no->setText("Serial No : " + QString::fromStdString(ser_no));
    ui->label_hw_ver->setText("HW: " + QString::fromStdString(hw_ver));
    ui->label_gain->setText(QString("gain (%1-%2):").arg(ptr_dev->gain_min()).arg(ptr_dev->gain_max()));

    id_device = id_miri;
    ui->push_button_start->setEnabled(true);

}
//---------------------------------------------------------------------------------------------------------------------------------
int main_window::start_dev()
{
    uint64_t rf_fraquency_hz;
    int gain;
    int err;
    rf_fraquency_hz = static_cast<uint64_t>(ui->spinBoxRF->text().toULong());
    gain = static_cast<uint8_t>(ui->spinBoxGain->value());
    if(ui->check_box_agc->isChecked()) gain = -1;
    err = ptr_dev->init(rf_fraquency_hz, gain);
    ui->text_log->insertPlainText("Init " + ptr_dev->dev_name() +": "  +
                                  QString::fromStdString(ptr_dev->error(err)) + "\n");
    if(err !=0) return err;
    ptr_dev->set_biastee(ui->checkBox_biastee->isChecked());
    ptr_dev->demodulator->set_fir(ui->comboBoxFIR->currentIndex());

    thread = new QThread;
    thread->setObjectName(ptr_dev->thread_name());
    ptr_dev->moveToThread(thread);
    connect(thread, SIGNAL(started()), ptr_dev, SLOT(start()));
    connect(this,SIGNAL(stop_device()),ptr_dev,SLOT(stop()),Qt::DirectConnection);
    connect(thread, SIGNAL(finished()), ptr_dev, SLOT(deleteLater()));
    connect(ptr_dev, SIGNAL(finished()), thread, SLOT(quit()),Qt::DirectConnection);
    connect(thread, SIGNAL(finished()), thread, SLOT(deleteLater()));
    connect(thread, SIGNAL(finished()), this, SLOT(finished_dev()));
    thread->start(QThread::TimeCriticalPriority);

    connect(ptr_dev, &rx_interface::status, this, &main_window::status_dev);
    connect(ptr_dev, &rx_interface::radio_frequency, this, &main_window::radio_frequency);
    connect(ptr_dev, &rx_interface::level_gain, this, &main_window::level_gain);
    connect(ptr_dev, &rx_interface::buffered, this, &main_window::update_buffered, Qt::QueuedConnection);
    connect(ui->spinBoxGain,SIGNAL(valueChanged(int)),ptr_dev,SLOT(set_gain_db(int)),Qt::DirectConnection);
    connect(ui->comboBoxFIR,SIGNAL(currentIndexChanged(int)),ptr_dev->demodulator,SLOT(set_fir(int)));

    return 0;
}
//---------------------------------------------------------------------------------------------------------------------------------
void main_window::status_dev(int _err)
{
    ui->text_log->insertPlainText("Status " + ptr_dev->dev_name() + ": "  +
                                  QString::fromStdString(ptr_dev->error(_err)) + "\n");
}
//---------------------------------------------------------------------------------------------------------------------------------
void main_window::finished_dev()
{
    ptr_dev = nullptr;
    thread = nullptr;
}
//---------------------------------------------------------------------------------------------------------------------------------
void main_window::update_buffered(int nbuffers, int totalbuffers, float level)
{
    ui->label_info_buffered->setText("buffered  :" + QString::number(nbuffers) + "/" +
        QString::number(totalbuffers) + " [" + QString::number(level, 'f' ,5) + "]");
    if(ptr_dev && enable_gain_updates)
        ptr_dev->update_gain_frequency_direct();
}
//---------------------------------------------------------------------------------------------------------------------------------
void main_window::on_push_button_start_clicked()
{
    ui->menu_open->setEnabled(false);
    if(start_dev() != 0) return;
    dvbt2 = ptr_dev->demodulator;
    for(int i = 1; i < ui->tab_widget->count(); ++i) ui->tab_widget->setTabEnabled(i, true);
    connect_info();
    ui->push_button_start->setEnabled(false);
    ui->spinBoxRF->setEnabled(false);
    ui->check_box_agc->setEnabled(false);
    ui->push_button_stop->setEnabled(true);
    enable_gain_updates = true;
}
//------------------------------------------------------------------------------------------------
void main_window::connect_info()
{
    connect(&dvbt2->p1_demodulator, &p1_symbol::bad_signal, this, &main_window::bad_signal);
    connect(&dvbt2->p2_demodulator, &p2_symbol::view_l1_presignalling, this, &main_window::view_l1_presignalling);
    connect(&dvbt2->p2_demodulator, &p2_symbol::view_l1_postsignalling, this, &main_window::view_l1_postsignalling);
    connect(&dvbt2->p2_demodulator, &p2_symbol::view_l1_dynamic, this, &main_window::view_l1_dynamic);
    connect(dvbt2, &dvbt2_demodulator::amount_plp, this, &main_window::amount_plp);
    connect(dvbt2->deinterleaver->qam, &llr_demapper::signal_noise_ratio,
            this, &main_window::signal_noise_ratio);
    connect(dvbt2->deinterleaver->qam->decoder->decoder->deheader, &bb_de_header::ts_stage,
            this, &main_window::ts_stage);
    qRegisterMetaType<bb_de_header::id_out>();
    connect(this, &main_window::set_out,
            dvbt2->deinterleaver->qam->decoder->decoder->deheader, &bb_de_header::set_out);
}
//------------------------------------------------------------------------------------------------
void main_window::bad_signal()
{
    if(ui->push_button_stop->isEnabled()) {
        on_push_button_stop_clicked();
        QMessageBox mes_box;
        mes_box.setText("Bad signal !\nСheck gain and frequency.");
        mes_box.exec();
    }
}
//------------------------------------------------------------------------------------------------
void main_window::on_push_button_stop_clicked()
{
    enable_gain_updates = false;
    disconnect_signals();
    disconnect_info();
    ui->tab_widget->setCurrentIndex(0);
    for(int i = 1; i < ui->tab_widget->count(); ++i) ui->tab_widget->setTabEnabled(i, false);
    ui->push_button_start->setEnabled(false);
    ui->push_button_stop->setEnabled(false);
    ui->push_button_ts_open_file->setEnabled(false);
    ui->push_button_ts_apply->setEnabled(false);
    emit stop_device();
    if(thread->isRunning()) thread->wait();
    ui->label_name->setText("Name :");
    ui->label_ser_no->setText("Serial No :");
    ui->label_hw_ver->setText("Hardware ver :");
    ui->label_info_rf->setText("radio frequency (Hz) : ");
    ui->label_info_gain->setText("gain reducton (dB) : ");
    ui->menu_open->setEnabled(true);
    ui->spinBoxRF->setEnabled(true);
    ui->check_box_agc->setEnabled(true);
}
//------------------------------------------------------------------------------------------------
void main_window::disconnect_info()
{
    disconnect(&dvbt2->p1_demodulator, &p1_symbol::bad_signal, this, &main_window::bad_signal);
    disconnect(&dvbt2->p2_demodulator, &p2_symbol::view_l1_presignalling, this, &main_window::view_l1_presignalling);
    disconnect(&dvbt2->p2_demodulator, &p2_symbol::view_l1_postsignalling, this, &main_window::view_l1_postsignalling);
    disconnect(&dvbt2->p2_demodulator, &p2_symbol::view_l1_dynamic, this, &main_window::view_l1_dynamic);
    disconnect(dvbt2, &dvbt2_demodulator::amount_plp, this, &main_window::amount_plp);
    disconnect(dvbt2->deinterleaver->qam, &llr_demapper::signal_noise_ratio,
               this, &main_window::signal_noise_ratio);
    disconnect(dvbt2->deinterleaver->qam->decoder->decoder->deheader, &bb_de_header::ts_stage,
               this, &main_window::ts_stage);
    disconnect(this, &main_window::set_out,
               dvbt2->deinterleaver->qam->decoder->decoder->deheader, &bb_de_header::set_out);
}
//------------------------------------------------------------------------------------------------
void main_window::radio_frequency(double _rf)
{
    ui->label_info_rf->setText("radio frequency (Hz) : " + QString::number(_rf, 'f', 0));
}
//------------------------------------------------------------------------------------------------
void main_window::level_gain(int _gain)
{
    QString str_gain = "";
    str_gain = "gain :   ";
    ui->label_info_gain->setText(str_gain + QString::number(_gain));
}
//------------------------------------------------------------------------------------------------
void main_window::on_tab_widget_currentChanged(int index)
{
    switch(index){
    case 1:
        disconnect_signals();
        connect(&dvbt2->p1_demodulator, &p1_symbol::replace_spectrograph,
                p1_spectrograph, &plot::replace_spectrograph);
        connect(&dvbt2->p1_demodulator, &p1_symbol::replace_constelation,
                p1_constelation, &plot::replace_constelation);
        dvbt2->p1_demodulator.enable_display(true);
        break;
    case 2:
        disconnect_signals();
        connect(&dvbt2->p2_demodulator, &p2_symbol::replace_spectrograph,
                p2_spectrograph, &plot::replace_spectrograph);
        connect(&dvbt2->p2_demodulator, &p2_symbol::replace_constelation,
                p2_constelation, &plot::replace_constelation);
        dvbt2->p2_demodulator.enable_display(true);
        break;
    case 3:
        disconnect_signals();
        connect(&dvbt2->data_demodulator, &data_symbol::replace_spectrograph,
                data_spectrograph, &plot::replace_spectrograph);
        connect(&dvbt2->data_demodulator, &data_symbol::replace_constelation,
                data_constelation, &plot::replace_constelation);
        dvbt2->data_demodulator.enable_display(true);
        break;
    case 4:
        disconnect_signals();
        connect(&dvbt2->fc_demod, &fc_symbol::replace_spectrograph,
                fc_spectrograph, &plot::replace_spectrograph);
        connect(&dvbt2->fc_demod, &fc_symbol::replace_constelation,
                fc_constelation, &plot::replace_constelation);
        dvbt2->fc_demod.enable_display(true);
        break;
    case 5:
        disconnect_signals();
        connect(dvbt2->deinterleaver, &time_deinterleaver::replace_constelation,
                qam_constelation, &plot::replace_constelation);
        connect(dvbt2->deinterleaver->qam->decoder, &ldpc_decoder::replace_oscilloscope,
                ldpc_stats, &plot::replace_oscilloscope);
        dvbt2->deinterleaver->enable_display(true);
        break;
    case 8:
        disconnect_signals();
        connect(&dvbt2->p1_demodulator, &p1_symbol::replace_oscilloscope,
                p1_correlation_oscilloscope, &plot::replace_oscilloscope);

        connect(&dvbt2->p2_demodulator, &p2_symbol::replace_oscilloscope,
                p2_equalizer_oscilloscope, &plot::replace_oscilloscope);

        connect(dvbt2, &dvbt2_demodulator::replace_null_indicator,
                frequency_offset, &plot::replace_null_indicator);
        dvbt2->p1_demodulator.enable_display(true);
        dvbt2->p2_demodulator.enable_display(true);
        dvbt2->enable_display(true);
        break;
    default:
        disconnect_signals();
    }
}
//------------------------------------------------------------------------------------------------
void main_window::disconnect_signals()
{
    dvbt2->p1_demodulator.enable_display(false);
    dvbt2->fc_demod.enable_display(false);
    dvbt2->data_demodulator.enable_display(false);
    dvbt2->p2_demodulator.enable_display(false);
    dvbt2->deinterleaver->enable_display(false);
    dvbt2->enable_display(false);
    disconnect(&dvbt2->p1_demodulator, &p1_symbol::replace_spectrograph,
            p1_spectrograph, &plot::replace_spectrograph);
    disconnect(&dvbt2->p1_demodulator, &p1_symbol::replace_constelation,
            p1_constelation, &plot::replace_constelation);
    disconnect(&dvbt2->p2_demodulator, &p2_symbol::replace_spectrograph,
            p2_spectrograph, &plot::replace_spectrograph);
    disconnect(&dvbt2->p2_demodulator, &p2_symbol::replace_constelation,
            p2_constelation, &plot::replace_constelation);
    disconnect(&dvbt2->data_demodulator, &data_symbol::replace_constelation,
            data_spectrograph, &plot::replace_constelation);
    disconnect(&dvbt2->data_demodulator, &data_symbol::replace_spectrograph,
            data_spectrograph, &plot::replace_spectrograph);
    disconnect(&dvbt2->data_demodulator, &data_symbol::replace_constelation,
            data_constelation, &plot::replace_constelation);
    disconnect(&dvbt2->fc_demod, &fc_symbol::replace_spectrograph,
            fc_spectrograph, &plot::replace_spectrograph);
    disconnect(&dvbt2->fc_demod, &fc_symbol::replace_constelation,
            fc_constelation, &plot::replace_constelation);
    disconnect(dvbt2->deinterleaver, &time_deinterleaver::replace_constelation,
            qam_constelation, &plot::replace_constelation);
    disconnect(&dvbt2->p1_demodulator, &p1_symbol::replace_oscilloscope,
            p1_correlation_oscilloscope, &plot::replace_oscilloscope);
    disconnect(&dvbt2->p2_demodulator, &p2_symbol::replace_oscilloscope,
            p2_equalizer_oscilloscope, &plot::replace_oscilloscope);
    disconnect(dvbt2, &dvbt2_demodulator::replace_null_indicator,
            frequency_offset, &plot::replace_null_indicator);
    disconnect(dvbt2->deinterleaver->qam->decoder, &ldpc_decoder::replace_oscilloscope,
            ldpc_stats, &plot::replace_oscilloscope);
}
//------------------------------------------------------------------------------------------------
void main_window::set_show_p2_symbol(int _id)
{
    switch(_id){
    case 0:
        dvbt2->p2_demodulator.id_show = p2_symbol::p2_l1_pre;
        break;
    case 1:
        dvbt2->p2_demodulator.id_show = p2_symbol::p2_l1_post;
        break;
    case 2:
        dvbt2->p2_demodulator.id_show = p2_symbol::p2_data;
        break;
    }
}
//------------------------------------------------------------------------------------------------
void main_window::amount_plp(int _num_plp)
{
    ui->combo_box_plp_id->clear();
    for(int i = 0; i < _num_plp; ++i) ui->combo_box_plp_id->addItem(QString::number(i));
    ui->combo_box_ts_plp->clear();
    for(int i = 0; i < _num_plp; ++i) ui->combo_box_ts_plp->addItem(QString::number(i));
}
//------------------------------------------------------------------------------------------------
void main_window::on_combo_box_plp_id_currentIndexChanged(int index)
{
    dvbt2->deinterleaver->idx_show_plp = index;
}
//------------------------------------------------------------------------------------------------
void main_window::signal_noise_ratio(float _snr)
{
    ui->label_snr->setText("SNR : " + QString::number(static_cast<double>(_snr),'f',2) + "dB");
}
//------------------------------------------------------------------------------------------------
void main_window::view_l1_presignalling(QString _text)
{
    ui->text_edit_l1_presignalling->clear();
    ui->text_edit_l1_presignalling->append(_text);
}
//------------------------------------------------------------------------------------------------
void main_window::view_l1_postsignalling(QString _text)
{
    ui->text_edit_l1_postsignalling->clear();
    ui->text_edit_l1_postsignalling->append(_text);
}
//------------------------------------------------------------------------------------------------
void main_window::view_l1_dynamic(QString _text, bool _force_update)
{
    static bool update = true;
    if(update || ui->check_box_auto_update_l1_dynamic->isChecked()) {
        update = false;
        ui->text_edit_l1_postsignalling_dynamic->clear();
        ui->text_edit_l1_postsignalling_dynamic->append(_text);
    }
    if(_force_update) update = true;
}
//------------------------------------------------------------------------------------------------
void main_window::ts_stage(QString _info)
{
    static QString old_stage = "";
    static int count = 0;
    QString info = "";
//    ui->text_edit_ts_stage->clear();
    if(_info != old_stage) {
        info = _info;
        old_stage = _info;
        count = 0;
    }
    else {
        ++count;
        info = _info + " (count.. " + QString::number(count) + ")";
    }
    ui->text_edit_ts_stage->append(info);
}
//------------------------------------------------------------------------------------------------
void main_window::on_push_button_ts_apply_clicked()
{
    bb_de_header::id_out id_current_out = bb_de_header::out_network;
    int num_port_udp = 7654;
    QString port = ui->line_edit_ts_udp_port->text();
    if(port != "") num_port_udp = port.toInt();
    else ui->line_edit_ts_udp_port->setText("7654");

    if(ui->radio_button_ts_file->isChecked()) id_current_out = bb_de_header::out_file;
    QString file_name = ui->line_edit_ts_file_name->text();

    QString plp = ui->combo_box_ts_plp->currentText();
    int need_plp = 0;
    need_plp = plp.toInt();

    emit set_out(id_current_out, num_port_udp, file_name, need_plp);

    ui->push_button_ts_apply->setEnabled(false);
}
//------------------------------------------------------------------------------------------------
void main_window::on_combo_box_ts_plp_currentIndexChanged(const QString &arg1)
{
    Q_UNUSED(arg1)
    ui->push_button_ts_apply->setEnabled(true);
}
//-----------------------------------------------------------------------------------------------
void main_window::on_radio_button_ts_net_toggled(bool checked)
{
    if(checked) {
        ui->line_edit_ts_udp_port->setEnabled(true);
        ui->push_button_ts_apply->setEnabled(true);
    }
    else {
        ui->line_edit_ts_udp_port->setEnabled(false);
    }
}
//------------------------------------------------------------------------------------------------
void main_window::on_line_edit_ts_udp_port_textChanged(const QString &arg1)
{
    Q_UNUSED(arg1)
    ui->push_button_ts_apply->setEnabled(true);
}
//------------------------------------------------------------------------------------------------
void main_window::on_radio_button_ts_file_toggled(bool checked)
{
    if(checked) ui->push_button_ts_open_file->setEnabled(true);
    else ui->push_button_ts_open_file->setEnabled(false);
}
//------------------------------------------------------------------------------------------------
void main_window::on_push_button_ts_open_file_clicked()
{
    QString file_name = "";
    file_name = QFileDialog::getSaveFileName(this);
    ui->line_edit_ts_file_name->setText(file_name);
    if(file_name != "") ui->push_button_ts_apply->setEnabled(true);
    else ui->radio_button_ts_net->setChecked(false);
}
//------------------------------------------------------------------------------------------------
void main_window::on_checkBox_biastee_toggled(bool checked)
{
    if(ptr_dev)
        ptr_dev->set_biastee(checked);
}
//------------------------------------------------------------------------------------------------
