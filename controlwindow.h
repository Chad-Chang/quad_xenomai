
#ifndef CONTROLWINDOW_H
#define CONTROLWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QThread>
#include <QPixmap>
#include <algorithm>
#include <qcustomplot.h>
#include <stdio.h>
#include <data_mutex.h>

using namespace std;
QT_BEGIN_NAMESPACE
namespace Ui { class Controlwindow;}
QT_END_NAMESPACE

class Controlwindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit Controlwindow(QWidget *parent = nullptr);
    ~Controlwindow();
    void Mutex_exchange();
    void Widget_update();
    void createPlot_3data(QCustomPlot *plot);
    void drawPlot_3data(QCustomPlot *plot, double data1, double data2, double data3);
    void Enable();//independent
    void homming();//independent
    void DOB();//independent
    void Admittance();//independent
    void ControlWord(uint16_t k);

    Ui::Controlwindow *ui;    
private slots: // signal -> slot event
    void on_Set_clicked(); // has to be define to avoid duplicate from internal library
    void updateWindow(); // dependent to Widget_update(),Mutex_exchange(),Enable(), homming()
    void on_init_encoder_clicked(); //independent
    void on_DOB_clicked();
    void on_Admittance_clicked();
    void on_Homming_clicked();   //independent
    void on_STOP_clicked(); //independent
    
  private:
    
    // Variable //
    Vector4d RW_r_posPgain;
    Vector4d RW_r_posIgain;
    Vector4d RW_r_posDgain;
    Vector4d RW_r_posD_cutoff;
    
    Vector4d RW_th_posPgain;
    Vector4d RW_th_posIgain;
    Vector4d RW_th_posDgain;
    Vector4d RW_th_posD_cutoff;
    
    Vector4d RW_r_velPgain;
    Vector4d RW_r_velIgain;
    Vector4d RW_r_velDgain;
    Vector4d RW_r_velD_cutoff;
    
    Vector4d RW_th_velPgain;
    Vector4d RW_th_velIgain; 
    Vector4d RW_th_velDgain;
    Vector4d RW_th_velD_cutoff;
    
    double Motor_current[NUMOFSLAVES];
    
    // RW state //
    double Motor_pos[NUMOFSLAVES];    
    double Motor_pos_init[NUMOFSLAVES];
    double Homming_input[NUMOFSLAVES];
    double Motor_Home_pos[NUMOFSLAVES];
    
    double key;
    double Plot_time_window_POS = 10.0;
    uint16_t k;
    
    // flag //
    bool Traj_on = false;
    bool Ctrl_on = false;
    int CtrlMode;
    bool Enc_init = false;
    bool DOB_init = false;
    bool admittance_init = false;

    bool DOB_checked = false;
    bool Admittance_checked = false;
    bool Homming_checked = false;
    bool Homming_clicked = false;
    bool DataLog_flag = false;
    bool stop = false;
    
    
};

#endif // CONTROLWINDOW_H
