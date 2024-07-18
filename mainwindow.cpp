#include "mainwindow.h"
#include "./ui_mainwindow.h"

#include <pthread.h>
#include "data_mutex.h"

#include <QTimer>
#include <QThread>

QSharedPointer<QCPAxisTickerTime> timeTicker(new QCPAxisTickerTime);


/*           UI variable         */
int8_t MODE_OF_OPERATION[NUMOFSLAVES];
double ref_pos[NUMOFSLAVES];
double ref_vel[NUMOFSLAVES];
double ref_current[NUMOFSLAVES];
double P_gain[NUMOFSLAVES];
double I_gain[NUMOFSLAVES];
double D_gain[NUMOFSLAVES];
double Target_torque[NUMOFSLAVES];
double CONTROLWORD[NUMOFSLAVES];

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
  ui->setupUi(this);

  data_mut_lock_timeout.tv_nsec = timeout_ns;
  data_mut_lock_timeout.tv_sec = 0;

  tim = new QTimer(this);
  connect(tim, SIGNAL(timeout()), this, SLOT(updateWindow()));
  tim->start(50);
  
  //*************** Plot setting -> creating template***************//
    //FL
  createPlot(ui->FL_r_pos_plot);
  createPlot(ui->FL_th_pos_plot);
  createPlot(ui->FL_current); 
    //FR
//  createPlot(ui->FR_r_pos_plot);
//  createPlot(ui->FR_th_pos_plot);
//  createPlot(ui->FR_current); 
    //RL
  createPlot(ui->RL_r_pos_plot);
  createPlot(ui->RL_th_pos_plot);
  createPlot(ui->RL_current); 
    //RR
  createPlot(ui->RR_r_pos_plot);
  createPlot(ui->RR_th_pos_plot);
  createPlot(ui->RR_current); 

}

void MainWindow::updateWindow()
{
    
  static QTime timeStart = QTime::currentTime();          // calculate two new data points:
  key = timeStart.msecsTo(QTime::currentTime()) / 1000.0; // time elapsed since start of demo, in seconds
  timeTicker->setTimeFormat("%m:%s");
  
  //*************** plot data ***************//
 
  //FL
 
  drawPlot(ui->FL_r_pos_plot, RW_r_pos[0], ref_r_pos[0], ui->FL_r_pos_autoscale) ; 
  ui->FL_r_pos_plot->yAxis->setRange(0, 0.5);
  drawPlot(ui->FL_th_pos_plot, RW_th_pos[0], ref_th_pos[0], ui->FL_th_pos_autoscale);
  ui->FL_th_pos_plot->yAxis->setRange(0, PI);
  drawPlot(ui->FL_current, Motor_current[7], Motor_current[8], ui->FL_current_autoscale);
  
  //FR
 
//  drawPlot(ui->FR_r_pos_plot, RW_r_pos[1], ref_r_pos[1], ui->FR_r_pos_autoscale) ; 
//  ui->FR_r_pos_plot->yAxis->setRange(0, 0.5);
//  drawPlot(ui->FR_th_pos_plot, RW_th_pos[1], ref_th_pos[1], ui->FR_th_pos_autoscale);
//  ui->RL_th_pos_plot->yAxis->setRange(0, PI);
//  drawPlot(ui->FR_current, Motor_current[0], Motor_current[1], ui->FR_current_autoscale);
  
  //RL
 
  drawPlot(ui->RL_r_pos_plot, RW_r_pos[2], ref_r_pos[2], ui->RL_r_pos_autoscale) ; 
  ui->RL_r_pos_plot->yAxis->setRange(0, 0.5);
  drawPlot(ui->RL_th_pos_plot, RW_th_pos[2], ref_th_pos[2], ui->RL_th_pos_autoscale);
  ui->RL_th_pos_plot->yAxis->setRange(0, PI);
  drawPlot(ui->RL_current, Motor_current[0], Motor_current[1], ui->RL_current_autoscale);
  
  //RR
 
  drawPlot(ui->RR_r_pos_plot, RW_r_pos[3], ref_r_pos[3], ui->RR_r_pos_autoscale) ; 
  ui->RR_r_pos_plot->yAxis->setRange(0, 0.5);
  drawPlot(ui->RR_th_pos_plot, RW_th_pos[3], ref_th_pos[3], ui->RR_th_pos_autoscale);
  ui->RR_th_pos_plot->yAxis->setRange(0, PI);
  drawPlot(ui->RR_current, Motor_current[2], Motor_current[3], ui->RR_current_autoscale);
  
  
  Widget_update();    
  Mutex_exchange();
  
  //// flag ////
  Enc_init = false;

}

void MainWindow::Widget_update(){
  


}

void MainWindow::drawPlot(QCustomPlot *plot, double data1, double data2, QCheckBox *autoscale){
  
  plot->graph(0)->addData(key,data1);
  plot->graph(1)->addData(key,data2);
  plot->xAxis->setRange(key,Plot_time_window_POS,Qt::AlignRight);

  if(autoscale->isChecked()) {
  plot->graph(0)->rescaleValueAxis(false,true);
  plot->graph(1)->rescaleValueAxis(false,true);
  }
  
  plot->graph(0)->data()->removeBefore(key-Plot_time_window_POS);
  plot->graph(1)->data()->removeBefore(key-Plot_time_window_POS);
  plot->replot();

}

void MainWindow::createPlot(QCustomPlot *plot) {
  plot->addGraph();
  plot->graph(0)->setPen(QPen(QColor(237, 237, 237)));
  plot->addGraph();
  plot->graph(1)->setPen(QPen(QColor(255, 246, 18)));
  plot->xAxis->setTicker(timeTicker);
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

MainWindow::~MainWindow() { delete ui; }


void MainWindow::Mutex_exchange(){

    int ret = pthread_mutex_timedlock(&data_mut,&data_mut_lock_timeout);
    if (!ret) {
      //// Receive at GUI ////
      sampling_time_ms = _M_sampling_time_ms;
      overrun_cnt = _M_overrun_cnt;
      WKC = _M_Ecat_WKC;
      expectedWKC = _M_Ecat_expectedWKC;
      
      _M_Enc_init = Enc_init;

      for (int i = 0; i < NUMOFSLAVES; i++) {

        //Motor State


        _M_CONTROLWORD[i] = CONTROLWORD[i];       
        Motor_current[i] =  _M_actual_current[i];
      }
      
      for (int i = 0; i < NUMOFLEGS; i++) {

        //Leg State

        RW_r_pos[i] = _M_RW_r_pos[i];
        RW_th_pos[i] = _M_RW_th_pos[i];
        ref_r_pos[i] = _M_ref_r_pos[i];
        ref_th_pos[i] = _M_ref_th_pos[i];
        
        r_pos_error[i] = _M_r_pos_error[i];
        th_pos_error[i] = _M_th_pos_error[i];
        
      }
      

    } else {
      QString str_errCode;
      str_errCode.setNum(ret);
    }
    pthread_mutex_unlock(&data_mut);

    QString str_sampling_time_ms;
    QString str_overrun_cnt;
    QString str_WKC;
    QString str_expectedWKC;

    QString str_status_report;

    QString str_statusword;
    QString str_modeofoperation_disp;

    str_sampling_time_ms.clear();
    str_overrun_cnt.clear();
    str_WKC.clear();
    str_expectedWKC.clear();

    str_sampling_time_ms.setNum(sampling_time_ms,'f',4);
    str_sampling_time_ms.prepend("Real-time task: ");
    str_sampling_time_ms.append("\tms sampling with ");

    str_overrun_cnt.setNum(overrun_cnt);
    str_overrun_cnt.append("\ttimes overrun.\t\t");

    str_expectedWKC.setNum(expectedWKC);
    str_expectedWKC.prepend("EtherCAT data frame: Expected workcounter= ");
    str_expectedWKC.append(", ");
    str_WKC.setNum(WKC);
    str_WKC.prepend("Workcounter= ");

    str_status_report.clear();
    str_status_report.append(str_sampling_time_ms).append(str_overrun_cnt).append(str_expectedWKC).append(str_WKC);

    ui->statusbar->showMessage(str_status_report);
}

void MainWindow::on_Controlword128_clicked() {
    for (int i = 0; i < NUMOFSLAVES; i++) {
      _M_MODE_OF_OPERATION[i] = 4;
      CONTROLWORD[i] = 128;
    }
}
void MainWindow::on_Controlword6_clicked() {
    for (int i = 0; i < NUMOFSLAVES; i++) {
        CONTROLWORD[i] = 6;
    }
}
void MainWindow::on_Controlword7_clicked() {
    for (int i = 0; i < NUMOFSLAVES; i++) {
        CONTROLWORD[i] = 7;
    }
}
void MainWindow::on_Controlword14_clicked() {
    for (int i = 0; i < NUMOFSLAVES; i++) {
        CONTROLWORD[i] = 14;
    }
}
void MainWindow::on_Controlword15_clicked() {
    for (int i = 0; i < NUMOFSLAVES; i++) {
        CONTROLWORD[i] = 15;
    }
}

void MainWindow::on_init_encoder_clicked()
{
    Enc_init = true;
}

