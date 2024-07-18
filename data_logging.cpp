#include "data_logging.h"

ofstream os_data;

data_logging::data_logging(Controlwindow* CW)
{
  cout << "Data Logging Construct" << endl;
  w_Ctrl = CW;
}

void data_logging::data_log() 
{
    QString LoggingPath = "/home/mcl/Experiment_DATA/";  // 경로 입력 
    
        if(DataLog_flag){
            int Dataidx = 0;
            if(os_data.is_open() == false){
                if(w_Ctrl->ui->LineEdit_LogFileName->text() == QString("Log Data file name")){ 
                    LoggingPath.append(QString("Quad_data%1.t xt").arg(File_idx++));
                }
                else LoggingPath.append(w_Ctrl->ui->LineEdit_LogFileName->text().append(QString(".txt")));
                os_data.open(LoggingPath.toStdString());
                isDataLogging = true;
////JOINT ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                os_data.width(10); os_data<<std::left<<QString("time : ").toStdString(); os_data.width(5); os_data<<std::left<< "Data";
                os_data.width(10); os_data<<std::left<<QString("FL_r_pos : ").toStdString(); os_data.width(5); os_data<<std::left<< "Data";// RW_pos[1];
                os_data.width(10); os_data<<std::left<<QString("FR_r_pos : ").toStdString(); os_data.width(5); os_data<<std::left<< "Data";
                os_data.width(10); os_data<<std::left<<QString("RL_r_pos : ").toStdString(); os_data.width(5); os_data<<std::left<< "Data";
                os_data.width(10); os_data<<std::left<<QString("RR_r_pos : ").toStdString(); os_data.width(5); os_data<<std::left<< "Data";
               
//                os_data.width(10); os_data<<std::left<<QString(" : ").toStdString(); os_data.width(5); os_data<<std::left<< "Data";
//                os_data.width(10); os_data<<std::left<<QString(" : ").toStdString(); os_data.width(5); os_data<<std::left<< "Data"; 
//                os_data.width(10); os_data<<std::left<<QString(" : ").toStdString(); os_data.width(5); os_data<<std::left<< "Data";

                os_data<<endl;
                }
                for(int i = 0; i<NUMOFSLAVES; i++){
                    if (i==0) {os_data.width(15); os_data<<std::left<<QString("%Mang_J%1").arg(i+1).toStdString();}
                    else {os_data.width(15); os_data<<std::left<<QString("Mang_J%1").arg(i+1).toStdString();}
                    }

            os_data<<endl;
        }
        else if(isDataLogging){
            os_data.close();
            std::cout<<" ================= DATA SAVE FINISHED!! ============="<<endl;
            isDataLogging = false;
            }
        }

        
void data_logging::exchange_mutex() {

     DataLog_flag = _M_DataLog_flag;
     
     for(int i = 0; i < NUMOFLEGS; i++)
      {
        RW_r_pos[i] =  _M_RW_r_pos[i];
        RW_th_pos[i] =  _M_RW_th_pos[i];
        RW_r_vel[i] =  _M_RW_r_vel[i];
        RW_th_vel[i] =  _M_RW_th_vel[i];
      }

}
