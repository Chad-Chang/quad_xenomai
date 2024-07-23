//#ifndef DATA_LOGGING_H
//#define DATA_LOGGING_H


//class data_logging
//{
//public:
//  data_logging();
//};

//#endif // DATA_LOGGING_H
#ifndef DATA_LOGGING_H
#define DATA_LOGGING_H

#include <data_mutex.h>
#include <controlwindow.h>
#include <ui_controlwindow.h>
#include <fstream>

class data_logging
{

private:
  bool DataLog_flag = false;
  bool isDataLogging = false;
  Controlwindow* w_Ctrl;
  int File_idx = 0;
  
  Vector4d RW_r_pos;
  Vector4d RW_th_pos;
  Vector4d RW_r_vel;
  Vector4d RW_th_vel;

public:
  data_logging(Controlwindow* CW);
  void data_log();
  void exchange_mutex();
 


  
  
};

#endif // DATA_LOGGING_H
