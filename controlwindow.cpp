#include "controlwindow.h"
#include "ui_controlwindow.h"
#include <data_mutex.h>

#include <QThread>
#include <QTimer>
#include <pthread.h>


QSharedPointer<QCPAxisTickerTime> timeTicker2(new QCPAxisTickerTime);

/*           UI variable         */
uint16_t  CONTROLWORD[NUMOFSLAVES];
int8_t mode_of_operation[NUMOFSLAVES] = {0,};
double Homming_gain = 30.0;
double Homming_t = 0;
double Homming_duration = 5; 


Controlwindow::Controlwindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::Controlwindow) {
  
  ui->setupUi(this);
  
  data_mut_lock_timeout.tv_nsec = timeout_ns;
  data_mut_lock_timeout.tv_sec = 0;

  tim = new QTimer(this);
  connect(tim, SIGNAL(timeout()), this, SLOT(updateWindow()));
  tim->start(50);
  
  //*************** Plot setting -> creating template***************//
    
    createPlot_3data(ui->FL_current);
    createPlot_3data(ui->FR_current);
    createPlot_3data(ui->RL_current);
    createPlot_3data(ui->RR_current);    
    
}

void Controlwindow::updateWindow() {
  
  static QTime timeStart = QTime::currentTime();          // calculate two new data points:
  key = timeStart.msecsTo(QTime::currentTime()) / 1000.0; // time elapsed since start of demo, in seconds
  timeTicker2->setTimeFormat("%m:%s");
  
  Widget_update();
  Mutex_exchange();
  Enable();
  homming();
   
  //cout << "Hip_pos: " << Motor_pos[1] << "\nKnee_pos: " << Motor_pos[2] << endl;
   
  //// Data Logging ////
  if(ui->logging_on->isChecked()){
  DataLog_flag = true;}
  else{
  DataLog_flag = false;}
  
  //// flag ////
  Enc_init = false;
  
  //*************** plot data ***************//
  if(ui->Tab->currentIndex() == 0)
  {
  drawPlot_3data(ui->FL_current, Motor_current[0], Motor_current[1], Motor_current[2]);
  drawPlot_3data(ui->FR_current, Motor_current[5], Motor_current[4], Motor_current[3]);
  drawPlot_3data(ui->RL_current, Motor_current[11], Motor_current[10], Motor_current[9]);
  drawPlot_3data(ui->RR_current, Motor_current[6], Motor_current[7], Motor_current[8]);
  
  }
  
}

void Controlwindow::ControlWord(uint16_t k)
{
      switch(CONTROLWORD[k]){
      case 0:
        CONTROLWORD[k] = (uint16_t) 128;
        break;
      case 128:
        CONTROLWORD[k] = (uint16_t) 6;
        break;
      case 6:
        CONTROLWORD[k] = (uint16_t) 7;
        break;
      case 7:
        CONTROLWORD[k] = (uint16_t) 14;
        break;
      case 14:
        CONTROLWORD[k] = (uint16_t) 15;
        break;
      }
}

void Controlwindow::createPlot_3data(QCustomPlot *plot) {

  plot->addGraph();
  plot->graph(0)->setPen(QPen(QColor(237, 237, 237)));
  plot->addGraph();
  plot->graph(1)->setPen(QPen(QColor(255, 246, 18)));
  plot->addGraph();
  plot->graph(2)->setPen(QPen(QColor(255, 24, 18)));
  plot->xAxis->setTicker(timeTicker2);
  plot->axisRect()->setupFullAxesBox();
  connect(plot->xAxis, SIGNAL(rangeChanged(QCPRange)), plot->xAxis2, SLOT(setRange(QCPRange)));
  connect(plot->yAxis, SIGNAL(rangeChanged(QCPRange)), plot->yAxis2, SLOT(setRange(QCPRange)));
  plot->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignLeft | Qt::AlignTop);

  plot->xAxis->setBasePen(QPen(Qt::white, 1));
  plot->yAxis->setBasePen(QPen(Qt::white, 1));
  plot->xAxis->setTickPen(QPen(Qt::white, 1));
  plot->yAxis->setTickPen(QPen(Qt::white, 1));
  plot->xAxis->setSubTickPen(QPen(Qt::white, 1));
  plot->yAxis->setSubTickPen(QPen(Qt::white, 1));
  plot->xAxis->setTickLabelColor(Qt::white);
  plot->yAxis->setTickLabelColor(Qt::white);
  plot->xAxis->grid()->setPen(QPen(QColor(140, 140, 140), 1, Qt::DotLine));
  plot->yAxis->grid()->setPen(QPen(QColor(140, 140, 140), 1, Qt::DotLine));
  plot->xAxis->grid()->setSubGridPen(QPen(QColor(80, 80, 80), 1, Qt::DotLine));
  plot->yAxis->grid()->setSubGridPen(QPen(QColor(80, 80, 80), 1, Qt::DotLine));
  plot->xAxis->grid()->setSubGridVisible(true);
  plot->yAxis->grid()->setSubGridVisible(true);
  plot->xAxis->grid()->setZeroLinePen(Qt::NoPen);
  plot->yAxis->grid()->setZeroLinePen(Qt::NoPen);
  plot->setBackground(QColor(25, 35, 45));
  plot->axisRect()->setBackground(QColor(25, 35, 45));
}

void Controlwindow::Widget_update(){
  
  if(ui -> traj_on ->isChecked()){Traj_on = true;}
  else {Traj_on = false;}
  if(ui -> ctrl_on ->isChecked()){Ctrl_on = true;}
  else {Ctrl_on = false;}
  ui->LineEdit_LogFileName->text();
  
}



void Controlwindow::homming()
{
  if(ui->Homming->isChecked()){Homming_checked = true;}
  else{Homming_checked = false;} // Checked Flag
}

void Controlwindow::DOB()
{
  if(ui->DOB->isChecked()){DOB_checked = true;}
  else{DOB_checked = false;} // Checked Flag
}

void Controlwindow::Admittance()
{
  if(ui->Admittance->isChecked()){Admittance_checked = true;}
  else{Admittance_checked = false;} // Checked Flag
}



void Controlwindow::on_Homming_clicked()
{   
    
    if(ui->Homming->isChecked())
    { 
      for(int i = 0; i < NUMOFSLAVES; i ++)
      {Motor_pos_init[i] = Motor_pos[i];}
    }

}

Controlwindow::~Controlwindow()
{
  delete ui;
}


void Controlwindow::on_Set_clicked() { // click event -> all gain are set at once

    ///////////////// Mode Select /////////////////
    if(ui->posctrl_mode->isChecked() && !ui->velctrl_mode->isChecked()){CtrlMode = 0;}    
    else if(!ui->posctrl_mode->isChecked() && ui->velctrl_mode->isChecked()){CtrlMode = 1;}
    else {
//    QMessageBox::critical(this, "Error", "Check the control mode!"); 
    QMessageBox msgBox;
        msgBox.setText("Check the control mode!");
        msgBox.setWindowTitle("Error");
        msgBox.setStyleSheet("QLabel{ color: white; }");
        msgBox.setIcon(QMessageBox::Critical);

        // Set background color to distinguish the text
        msgBox.setStyleSheet("QMessageBox { background-color: black; } QLabel { color: white; }");

        msgBox.exec();
    return;
    }
    
    /****************** Gain ******************/
    
    //////////////////// Position ////////////////////
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


void Controlwindow::drawPlot_3data (QCustomPlot *plot, double data1, double data2, double data3){
  
  plot->graph(0)->addData(key,data1);
  plot->graph(1)->addData(key,data2);
  plot->graph(2)->addData(key,data3);
  plot->xAxis->setRange(key,Plot_time_window_POS,Qt::AlignRight);

//  plot->graph(0)->rescaleValueAxis(false,true);
//  plot->graph(1)->rescaleValueAxis(false,true);
//  plot->graph(2)->rescaleValueAxis(false,true);
  
  plot->graph(0)->data()->removeBefore(key-Plot_time_window_POS);
  plot->graph(1)->data()->removeBefore(key-Plot_time_window_POS);
  plot->graph(2)->data()->removeBefore(key-Plot_time_window_POS);
  plot->replot();

}
void Controlwindow::on_init_encoder_clicked()
{
    Enc_init = true;
    cout << "================initiate encoder =============r"<<endl;
}

void Controlwindow::on_DOB_clicked()
{
    DOB_init = true;
    cout << "=============initiate DOB=============== "<<endl;
}

void Controlwindow::on_Admittance_clicked()
{
    admittance_init = true;
    cout << "=============initiate admittance=============== "<<endl;
}



void Controlwindow::on_STOP_clicked()
{
    stop = true;
}

void Controlwindow::Enable()
{
  for(int i = 0; i < NUMOFSLAVES; i ++)
    {
      mode_of_operation[i] = 4;
    }
  if(ui->servo_on->isChecked())
  {
    ui->FLHAA->setChecked(true);
    ui->FLHIP->setChecked(true);
    ui->FLKNEE->setChecked(true);
    ui->FRHAA->setChecked(true);
    ui->FRHIP->setChecked(true);
    ui->FRKNEE->setChecked(true);
    ui->RLHAA->setChecked(true);
    ui->RLHIP->setChecked(true);
    ui->RLKNEE->setChecked(true);
    ui->RRHAA->setChecked(true);
    ui->RRHIP->setChecked(true);
    ui->RRKNEE->setChecked(true);    
    ui->WL->setChecked(true);    
    ui->WR->setChecked(true);        
  } 
  if(ui->servo_off->isChecked())
  {
    ui->FLHAA->setChecked(false);
    ui->FLHIP->setChecked(false);
    ui->FLKNEE->setChecked(false);
    ui->FRHAA->setChecked(false);
    ui->FRHIP->setChecked(false);
    ui->FRKNEE->setChecked(false);
    ui->RLHAA->setChecked(false);
    ui->RLHIP->setChecked(false);
    ui->RLKNEE->setChecked(false);
    ui->RRHAA->setChecked(false);
    ui->RRHIP->setChecked(false);
    ui->RRKNEE->setChecked(false);  
  }
  if(ui->FLHAA->isChecked()){ControlWord(0);}else{CONTROLWORD[0]=0;}
  if(ui->FLHIP->isChecked()){ControlWord(1);}else{CONTROLWORD[1]=0;}
  if(ui->FLKNEE->isChecked()){ControlWord(2);}else{CONTROLWORD[2]=0;}
  if(ui->FRHAA->isChecked()){ControlWord(5);}else{CONTROLWORD[5]=0;}
  if(ui->FRHIP->isChecked()){ControlWord(4);}else{CONTROLWORD[4]=0;}
  if(ui->FRKNEE->isChecked()){ControlWord(3);}else{CONTROLWORD[3]=0;}
  if(ui->RLHAA->isChecked()){ControlWord(11);}else{CONTROLWORD[11]=0;}
  if(ui->RLHIP->isChecked()){ControlWord(10);}else{CONTROLWORD[10]=0;}
  if(ui->RLKNEE->isChecked()){ControlWord(9);}else{CONTROLWORD[9]=0;}
  if(ui->RRHAA->isChecked()){ControlWord(6);}else{CONTROLWORD[6]=0;}
  if(ui->RRHIP->isChecked()){ControlWord(7);}else{CONTROLWORD[7]=0;}
  if(ui->RRKNEE->isChecked()){ControlWord(8);}else{CONTROLWORD[8]=0;}
  if(ui->WL->isChecked()){ControlWord(12);}else{CONTROLWORD[12]=0;}
  if(ui->WR->isChecked()){ControlWord(13);}else{CONTROLWORD[13]=0;}
  
  
}

void Controlwindow::Mutex_exchange()
{
    //data_mut : intialized in main.cpp
    int ret = pthread_mutex_timedlock(&data_mut,&data_mut_lock_timeout);
    
    if (!ret) {
      
      for (int i = 0; i < NUMOFSLAVES; i++) {
        //Motor State
        _M_MODE_OF_OPERATION[i] = mode_of_operation[i];
        _M_CONTROLWORD[i] = CONTROLWORD[i];
        Motor_current[i] = _M_actual_current[i];
        Motor_pos[i] = _M_motor_position[i];
        _M_Motor_pos_init[i] = Motor_pos_init[i];
        
        
      }
      
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
        
        _M_ctrl_mode = Ctrl_on;
        
      }
      
      //// Flag ////
      _M_Traj_ON = Traj_on;
      _M_Ctrl_on = Ctrl_on;
      _M_stop = stop;
      _M_Enc_init = Enc_init;
      
      _M_DOB_on = Admittance_checked;
      _M_admittance_on = Admittance_checked;
      
      _M_Homming_checked = Homming_checked;
      _M_Homming_clicked = Homming_clicked;
      _M_DataLog_flag = DataLog_flag;
      
    } else {
      QString str_errCode;
      str_errCode.setNum(ret);
    }
    pthread_mutex_unlock(&data_mut);
}

