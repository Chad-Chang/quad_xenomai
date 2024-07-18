#include "controlwindow.h"
#include "ui_controlwindow.h"
#include <data_mutex.h>

#include <QThread>
#include <QTimer>
#include <pthread.h>


Controlwindow::Controlwindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::Controlwindow) {
  
  ui->setupUi(this);
  
    data_mut_lock_timeout.tv_nsec = timeout_ns;
  data_mut_lock_timeout.tv_sec = 0;

  tim = new QTimer(this);
  connect(tim, SIGNAL(timeout()), this, SLOT(updateWindow()));
  tim->start(50);
 
}

void Controlwindow::updateWindow() {
  Widget_update();
  Mutex_exchange();
}

// 
void Controlwindow::Widget_update(){
  
  if(ui -> traj_on ->isChecked()){Traj_on = true;}
  else {Traj_on = false;}
  
  

}

Controlwindow::~Controlwindow()
{
    delete ui;
}

void Controlwindow::Mutex_exchange()
{

    int ret = pthread_mutex_timedlock(&data_mut,&data_mut_lock_timeout);
    if (!ret) {
      for (int i = 0; i < 4; i++) {
        _M_RW_r_posPgain[i] = RW_r_posPgain[i];
        _M_RW_r_posDgain[i] = RW_r_posDgain[i];
        _M_RW_r_posIgain[i] = RW_r_posIgain[i];
        _M_RW_r_posD_cutoff[i] = RW_r_posD_cutoff[i];
        _M_RW_th_posPgain[i] = RW_th_posPgain[i];
        _M_RW_th_posIgain[i] = RW_th_posIgain[i];
        _M_RW_th_posDgain[i] = RW_th_posDgain[i];
        _M_RW_th_posD_cutoff[i] = RW_th_posD_cutoff[i];
        
        _M_RW_r_velPgain[i] = RW_r_posPgain[i];
        _M_RW_r_velDgain[i] = RW_r_posDgain[i];
        _M_RW_r_velIgain[i] = RW_r_posIgain[i];
        _M_RW_r_velD_cutoff[i] = RW_r_posD_cutoff[i];
        _M_RW_th_velPgain[i] = RW_th_posPgain[i];
        _M_RW_th_velIgain[i] = RW_th_posIgain[i];
        _M_RW_th_velDgain[i] = RW_th_posDgain[i];
        _M_RW_th_velD_cutoff[i] = RW_th_posD_cutoff[i];
        
        _M_traj_ON = Traj_on; 
        //_M_ctrl_mode
        
      }
      // Mutex exchange

    } else {
      QString str_errCode;
      str_errCode.setNum(ret);
    }
    pthread_mutex_unlock(&data_mut);
}


void Controlwindow::on_Set_clicked() {

    ///////////////// Mode Select /////////////////
    if(ui->posctrl_mode->isChecked() && !ui->velctrl_mode->isChecked()){CtrlMode = 0;}    
    else if(!ui->posctrl_mode->isChecked() && ui->velctrl_mode->isChecked()){CtrlMode = 1;}
    else {QMessageBox::critical(this, "Error", "Check the control mode!"); return;}    

    /****************** Gain ******************/
    
    //////////////////// Position ////////////////////
    QMessageBox::critical(this, "Error", "Check the control mode!");
    /**** r direction ****/
    RW_r_posPgain[0] = ui->FL_r_posPgain->value();
    RW_r_posIgain[0] = ui->FL_r_posIgain->value();
    RW_r_posDgain[0] = ui->FL_r_posDgain->value();
    
    RW_r_posPgain[1] = ui->FR_r_posPgain->value();
    RW_r_posIgain[1] = ui->FR_r_posIgain->value();
    RW_r_posDgain[1] = ui->FR_r_posDgain->value();
    
    RW_r_posPgain[2] = ui->RL_r_posPgain->value();
    RW_r_posIgain[2] = ui->RL_r_posIgain->value();
    RW_r_posDgain[2] = ui->RL_r_posDgain->value();
    
    RW_r_posPgain[3] = ui->RR_r_posPgain->value();
    RW_r_posIgain[3] = ui->RR_r_posIgain->value();
    RW_r_posDgain[3] = ui->RR_r_posDgain->value();

    /**** th direction ****/
    RW_th_posPgain[0] = ui->FL_th_posPgain->value();
    RW_th_posIgain[0] = ui->FL_th_posIgain->value();
    RW_th_posDgain[0] = ui->FL_th_posDgain->value();
    
    RW_th_posPgain[1] = ui->FR_th_posPgain->value();
    RW_th_posIgain[1] = ui->FR_th_posIgain->value();
    RW_th_posDgain[1] = ui->FR_th_posDgain->value();
    
    RW_th_posPgain[2] = ui->RL_th_posPgain->value();
    RW_th_posIgain[2] = ui->RL_th_posIgain->value();
    RW_th_posDgain[2] = ui->RL_th_posDgain->value();
    
    RW_th_posPgain[3] = ui->RR_th_posPgain->value();
    RW_th_posIgain[3] = ui->RR_th_posIgain->value();
    RW_th_posDgain[3] = ui->RR_th_posDgain->value();

    /************** Cut off **************/

    /**** r direction ****/
    RW_r_posD_cutoff[0] = ui->FL_r_posD_cutoff->value();
    RW_r_posD_cutoff[1] = ui->FR_r_posD_cutoff->value();
    RW_r_posD_cutoff[2] = ui->RL_r_posD_cutoff->value();
    RW_r_posD_cutoff[3] = ui->RR_r_posD_cutoff->value();

    /**** th direction ****/
    RW_th_posD_cutoff[0] = ui->FL_th_posD_cutoff->value();
    RW_th_posD_cutoff[1] = ui->FR_th_posD_cutoff->value();
    RW_th_posD_cutoff[2] = ui->RL_th_posD_cutoff->value();
    RW_th_posD_cutoff[3] = ui->RR_th_posD_cutoff->value();
    
    //////////////////// Velocity ////////////////////
    
    /**** r direction ****/
    RW_r_velPgain[0] = ui->FL_r_velPgain->value();
    RW_r_velIgain[0] = ui->FL_r_velIgain->value();
    RW_r_velDgain[0] = ui->FL_r_velDgain->value();
    
    RW_r_velPgain[1] = ui->FR_r_velPgain->value();
    RW_r_velIgain[1] = ui->FR_r_velIgain->value();
    RW_r_velDgain[1] = ui->FR_r_velDgain->value();
    
    RW_r_velPgain[2] = ui->RL_r_velPgain->value();
    RW_r_velIgain[2] = ui->RL_r_velIgain->value();
    RW_r_velDgain[2] = ui->RL_r_velDgain->value();
    
    RW_r_velPgain[3] = ui->RR_r_velPgain->value();
    RW_r_velIgain[3] = ui->RR_r_velIgain->value();
    RW_r_velDgain[3] = ui->RR_r_velDgain->value();

    /**** th direction ****/
    RW_th_velPgain[0] = ui->FL_th_velPgain->value();
    RW_th_velIgain[0] = ui->FL_th_velIgain->value();
    RW_th_velDgain[0] = ui->FL_th_velDgain->value();
    
    RW_th_velPgain[1] = ui->FR_th_velPgain->value();
    RW_th_velIgain[1] = ui->FR_th_velIgain->value();
    RW_th_velDgain[1] = ui->FR_th_velDgain->value();
    
    RW_th_velPgain[2] = ui->RL_th_velPgain->value();
    RW_th_velIgain[2] = ui->RL_th_velIgain->value();
    RW_th_velDgain[2] = ui->RL_th_velDgain->value();
    
    RW_th_velPgain[3] = ui->RR_th_velPgain->value();
    RW_th_velIgain[3] = ui->RR_th_velIgain->value();
    RW_th_velDgain[3] = ui->RR_th_velDgain->value();

    /************** Cut off **************/

    /**** r direction ****/
    RW_r_velD_cutoff[0] = ui->FL_r_velD_cutoff->value();
    RW_r_velD_cutoff[1] = ui->FR_r_velD_cutoff->value();
    RW_r_velD_cutoff[2] = ui->RL_r_velD_cutoff->value();
    RW_r_velD_cutoff[3] = ui->RR_r_velD_cutoff->value();

    /**** th direction ****/
    RW_th_velD_cutoff[0] = ui->FL_th_velD_cutoff->value();
    RW_th_velD_cutoff[1] = ui->FR_th_velD_cutoff->value();
    RW_th_velD_cutoff[2] = ui->RL_th_velD_cutoff->value();
    RW_th_velD_cutoff[3] = ui->RR_th_velD_cutoff->value();
}



