#include "mainwindow.h"
#include "./ui_mainwindow.h"

#include <pthread.h>
#include "data_mutex.h"

#include <QTimer>
#include <QThread>


QSharedPointer<QCPAxisTickerTime> timeTicker(new QCPAxisTickerTime);


///*           UI variable         */
double ref_pos[NUMOFSLAVES];
double ref_vel[NUMOFSLAVES];
double ref_current[NUMOFSLAVES];
double P_gain[NUMOFSLAVES];
double I_gain[NUMOFSLAVES];
double D_gain[NUMOFSLAVES];
double Target_torque[NUMOFSLAVES];


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
  createPlot(ui->FL_r_plot);
  createPlot(ui->FL_th_plot);
  createPlot(ui->FL_current); 
    //FR
  createPlot(ui->FR_r_plot);
  createPlot(ui->FR_th_plot);
  createPlot(ui->FR_current); 
    //RL
  createPlot(ui->RL_r_plot);
  createPlot(ui->RL_th_plot);
  createPlot(ui->RL_current); 
    //RR
  createPlot(ui->RR_r_plot);
  createPlot(ui->RR_th_plot);
  createPlot(ui->RR_current); 
  
 
  createPlot(ui->FL_vel_r_plot);
  createPlot(ui->FR_vel_r_plot);
  createPlot(ui->RR_vel_r_plot);
  createPlot(ui->RL_vel_r_plot);
  
  createPlot(ui->FL_vel_th_plot);
  createPlot(ui->FR_vel_th_plot);
  createPlot(ui->RR_vel_th_plot);
  createPlot(ui->RL_vel_th_plot);
  
  //FL-jnt
  createPlot(ui->jnt0);
  createPlot(ui->jnt1);
  createPlot(ui->jnt2);
  //FR-jnt
  createPlot(ui->jnt3);
  createPlot(ui->jnt4);
  createPlot(ui->jnt5);
  //RR-jnt
  createPlot(ui->jnt6);
  createPlot(ui->jnt7);
  createPlot(ui->jnt8);
  //RL-jnt
  createPlot(ui->jnt9);
  createPlot(ui->jnt10);
  createPlot(ui->jnt11);
  //waist
  createPlot(ui->jnt12);
  createPlot(ui->jnt13);
  
  //DOB
  createPlot(ui->DOB_tau_m);
  createPlot(ui->DOB_tau_b);
  
  //DOB
  createPlot(ui->FOB_r_force);
  createPlot(ui->FOB_force_x);
  createPlot(ui->FOB_force_y);
  createPlot(ui->FOB_force_z);
  createPlot(ui->admittance_r);
  createPlot(ui->r_pos_plot);
  
  
  
  

}

void MainWindow::updateWindow()
{
    
  static QTime timeStart = QTime::currentTime();          // calculate two new data points:
  key = timeStart.msecsTo(QTime::currentTime()) / 1000.0; // time elapsed since start of demo, in seconds
  timeTicker->setTimeFormat("%m:%s");
  
    
  Widget_update();    
  Mutex_exchange();
  
  //*************** plot data ***************//
  
  
  if(ui->Select_data->currentIndex() == 0)
  {
    //FL
    drawPlot(ui->FL_r_plot, RW_r_pos[0], ref_r_pos[0], ui->FL_r_autoscale) ;
    ui->FL_r_plot->yAxis->setRange(0, 0.5);
    drawPlot(ui->FL_th_plot, RW_th_pos[0], ref_th_pos[0], ui->FL_th_autoscale);
    ui->FL_th_plot->yAxis->setRange(0, PI);
    
    //FR
    drawPlot(ui->FR_r_plot, RW_r_pos[1], ref_r_pos[1], ui->FR_r_autoscale) ;
    ui->FR_r_plot->yAxis->setRange(0, 0.5);
    drawPlot(ui->FR_th_plot, RW_th_pos[1], ref_th_pos[1], ui->FR_th_autoscale);
    ui->RL_th_plot->yAxis->setRange(0, PI);
    
    //RL
    drawPlot(ui->RL_r_plot, RW_r_pos[2], ref_r_pos[2], ui->RL_r_autoscale) ;
    ui->RL_r_plot->yAxis->setRange(0, 0.5);
    drawPlot(ui->RL_th_plot, RW_th_pos[2], ref_th_pos[2], ui->RL_th_autoscale);
    ui->RL_th_plot->yAxis->setRange(0, PI);
    
    //RR
    drawPlot(ui->RR_r_plot, RW_r_pos[3], ref_r_pos[3], ui->RR_r_autoscale) ;
    ui->RR_r_plot->yAxis->setRange(0, 0.5);
    drawPlot(ui->RR_th_plot, RW_th_pos[3], ref_th_pos[3], ui->RR_th_autoscale);
    ui->RR_th_plot->yAxis->setRange(0, PI);
  }
  else if(ui->Select_data->currentIndex() == 1)
  {
    //FL
    drawPlot(ui->FL_r_plot, RW_r_vel[0], ref_r_vel[0], ui->FL_r_autoscale) ;
    ui->FL_r_plot->yAxis->setRange(0, 0.5);
    drawPlot(ui->FL_th_plot, RW_th_vel[0], ref_th_vel[0], ui->FL_th_autoscale);
    ui->FL_th_plot->yAxis->setRange(0, PI);
    
    //FR
    drawPlot(ui->FR_r_plot, RW_r_vel[1], ref_r_vel[1], ui->FR_r_autoscale) ;
    ui->FR_r_plot->yAxis->setRange(0, 0.5);
    drawPlot(ui->FR_th_plot, RW_th_vel[1], ref_th_vel[1], ui->FR_th_autoscale);
    ui->RL_th_plot->yAxis->setRange(0, PI);
    
    //RL
    drawPlot(ui->RL_r_plot, RW_r_vel[2], ref_r_vel[2], ui->RL_r_autoscale) ;
    ui->RL_r_plot->yAxis->setRange(0, 0.5);
    drawPlot(ui->RL_th_plot, RW_th_vel[2], ref_th_vel[2], ui->RL_th_autoscale);
    ui->RL_th_plot->yAxis->setRange(0, PI);
    
    //RR
    drawPlot(ui->RR_r_plot, RW_r_vel[3], ref_r_vel[3], ui->RR_r_autoscale) ;
    ui->RR_r_plot->yAxis->setRange(0, 0.5);
    drawPlot(ui->RR_th_plot, RW_th_vel[3], ref_th_vel[3], ui->RR_th_autoscale);
    ui->RR_th_plot->yAxis->setRange(0, PI);
    
    
    //jnt
    drawPlot(ui->jnt0, 0,0, ui->jnt0_autoscale) ;
    ui->jnt0->yAxis->setRange(0, 0.5);
    drawPlot(ui->jnt1, 0,0, ui->jnt1_autoscale) ;
    ui->jnt1->yAxis->setRange(0, 0.5);
    drawPlot(ui->jnt2, 0,0, ui->jnt2_autoscale) ;
    ui->jnt2->yAxis->setRange(0, 0.5);
    drawPlot(ui->jnt3, 0,0, ui->jnt3_autoscale) ;
    ui->jnt3->yAxis->setRange(0, 0.5);
    drawPlot(ui->jnt4, 0,0, ui->jnt4_autoscale) ;
    ui->jnt4->yAxis->setRange(0, 0.5);
    drawPlot(ui->jnt5, 0,0, ui->jnt5_autoscale) ;
    ui->jnt5->yAxis->setRange(0, 0.5);
    drawPlot(ui->jnt6, 0,0, ui->jnt6_autoscale) ;
    ui->jnt6->yAxis->setRange(0, 0.5);
    drawPlot(ui->jnt7, 0,0, ui->jnt7_autoscale) ;
    ui->jnt7->yAxis->setRange(0, 0.5);
    drawPlot(ui->jnt8, 0,0, ui->jnt8_autoscale) ;
    ui->jnt8->yAxis->setRange(0, 0.5);
    drawPlot(ui->jnt9, 0,0, ui->jnt9_autoscale) ;
    ui->jnt9->yAxis->setRange(0, 0.5);
    drawPlot(ui->jnt10, 0,0, ui->jnt10_autoscale) ;
    ui->jnt10->yAxis->setRange(0, 0.5);
    drawPlot(ui->jnt11, 0,0, ui->jnt11_autoscale) ;
    ui->jnt11->yAxis->setRange(0, 0.5);
    drawPlot(ui->jnt12, 0,0, ui->jnt12_autoscale) ;
    ui->jnt12->yAxis->setRange(0, 0.5);
    drawPlot(ui->jnt13, 0,0, ui->jnt13_autoscale) ;
    ui->jnt13->yAxis->setRange(0, 0.5);
    
    // DOB 
    drawPlot(ui->jnt0, 0,0, ui->jnt0_autoscale) ;
    ui->jnt0->yAxis->setRange(0, 0.5);
    drawPlot(ui->jnt0, 0,0, ui->jnt0_autoscale) ;
    ui->jnt0->yAxis->setRange(0, 0.5);
    
    //FOB
    drawPlot(ui->FOB_r_force, 0,0, ui->FOB_r_autoscale) ;
    ui->FOB_r_force->yAxis->setRange(0, 0.5);
    drawPlot(ui->FOB_force_x, 0,0, ui->FOB_f_x_autoscale) ;
    ui->FOB_force_x->yAxis->setRange(0, 0.5);
    drawPlot(ui->FOB_force_y, 0,0, ui->FOB_f_y_autoscale) ;
    ui->FOB_force_y->yAxis->setRange(0, 0.5);
    drawPlot(ui->FOB_force_z, 0,0, ui->FOB_f_z_autoscale) ;
    ui->FOB_force_z->yAxis->setRange(0, 0.5);
    
    drawPlot(ui->admittance_r, 0,0, ui->admittance_r_autoscale) ;
    ui->admittance_r->yAxis->setRange(0, 0.5);
    drawPlot(ui->r_pos_plot, 0,0, ui->pos_r_autoscale) ;
    ui->r_pos_plot->yAxis->setRange(0, 0.5);
    

  createPlot(ui->FOB_r_force);
  createPlot(ui->FOB_force_x);
  createPlot(ui->FOB_force_y);
  createPlot(ui->FOB_force_z);
  createPlot(ui->admittance_r);
  createPlot(ui->r_pos_plot);
    
  }
  
  if(ui->Select_data_2->currentIndex() == 0)
  {
  drawPlot(ui->FL_current, Motor_current[1], Motor_current[2], ui->FL_current_autoscale);
  drawPlot(ui->FR_current, Motor_current[4], Motor_current[3], ui->FR_current_autoscale);
  drawPlot(ui->RL_current, Motor_current[11], Motor_current[10], ui->RL_current_autoscale);
  drawPlot(ui->RR_current, Motor_current[8], Motor_current[9], ui->RR_current_autoscale);
  }
  else if(ui->Select_data_2->currentIndex() == 1)
  {
  // DOB data plot
  }
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

void MainWindow::drawPlot_3data(QCustomPlot *plot, double data1, double data2, double data3, QCheckBox *autoscale){
  
  plot->graph(0)->addData(key,data1);
  plot->graph(1)->addData(key,data2);
  plot->graph(2)->addData(key,data3);
  plot->xAxis->setRange(key,Plot_time_window_POS,Qt::AlignRight);

  if(autoscale->isChecked()) {
  plot->graph(0)->rescaleValueAxis(false,true);
  plot->graph(1)->rescaleValueAxis(false,true);
  plot->graph(2)->rescaleValueAxis(false,true);
  }
  
  plot->graph(0)->data()->removeBefore(key-Plot_time_window_POS);
  plot->graph(1)->data()->removeBefore(key-Plot_time_window_POS);
  plot->graph(2)->data()->removeBefore(key-Plot_time_window_POS);
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

void MainWindow::createPlot_3data(QCustomPlot *plot) {
  plot->addGraph();
  plot->graph(0)->setPen(QPen(QColor(237, 237, 237)));
  plot->addGraph();
  plot->graph(1)->setPen(QPen(QColor(255, 246, 18)));
  plot->addGraph();
  plot->graph(2)->setPen(QPen(QColor(255, 24, 18)));
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
      
     
      for (int i = 0; i < NUMOFSLAVES; i++) {

        //Motor State
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
      
        RW_r_vel[i] = _M_RW_r_vel[i];
        RW_th_vel[i] = _M_RW_th_vel[i];
        ref_r_vel[i] = _M_ref_r_vel[i];
        ref_th_vel[i] = _M_ref_th_vel[i];
        
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


