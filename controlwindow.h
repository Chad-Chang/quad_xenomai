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

namespace Ui { class Controlwindow;}

class Controlwindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit Controlwindow(QWidget *parent = nullptr);
    ~Controlwindow();
    void Mutex_exchange();
    void Widget_update();
    
  
private slots:
    void on_Set_clicked();
    void updateWindow();
    
    
  private:
    Ui::Controlwindow *ui;
    
    
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
    
    double key;
    
    // flag //
    bool Traj_on = false;
    int CtrlMode;
    
};

#endif // CONTROLWINDOW_H
