//#ifndef MAINWINDOW_H
//#define MAINWINDOW_H

//#include <QMainWindow>

//QT_BEGIN_NAMESPACE
//namespace Ui { class MainWindow; }
//QT_END_NAMESPACE

//class MainWindow : public QMainWindow
//{
//  Q_OBJECT
  
//public:
//  MainWindow(QWidget *parent = nullptr);
//  ~MainWindow();
  
//private:
//  Ui::MainWindow *ui;
//};
//#endif // MAINWINDOW_H


#ifndef MAINWINDOW_H
#define MAINWINDOW_H

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
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void createPlot(QCustomPlot *plot);
    void drawPlot(QCustomPlot *plot, double data1, double data2, QCheckBox *autoscale);
    void createPlot_3data(QCustomPlot *plot);
    void drawPlot_3data(QCustomPlot *plot, double data1, double data2, double data3,QCheckBox *autoscale);
    void Widget_update();
    void Mutex_exchange();
    
    
private:
    Ui::MainWindow *ui;
    double sampling_time_ms = 0.0;
    int cnt_ID = 0;
    int overrun_cnt = 0;
    int WKC = 0;
    int expectedWKC = 0;
    double key;
    double Plot_time_window_POS = 10.0;
    
    double ref_r_pos[4];
    double ref_th_pos[4];
    double RW_r_pos[4];
    double RW_th_pos[4];
    
    double ref_r_vel[4];
    double ref_th_vel[4];
    double RW_r_vel[4];
    double RW_th_vel[4];
    
    double Motor_pos[NUMOFSLAVES];
    double Motor_current[NUMOFSLAVES];
    
    double r_pos_error[4];
    double th_pos_error[4];
    
       
private slots:
  void updateWindow();

};
#endif // MAINWINDOW_H
