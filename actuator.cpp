#include "actuator.h"
#include <iostream>
#include <stdio.h>
using namespace std;

Actuator::Actuator(int Motor_num, double motor_jig_pos) {
  Motor_Num = Motor_num; // /////////////////////////////////////////////// ZZi reum
//  Motor_pos = motor_init_pos;
  
  Motor_jig_pos = motor_jig_pos;
  
  QString ReadPath = "/home/mcl/Documents/RT_ECAT_MASTER_ELMO_GOLD/initial_encoder/";
  ReadPath.append(QString("Motor %1.txt").arg(Motor_Num));
  
  os_loadinit.open(ReadPath.toStdString());
  os_loadinit >> Motor_pos_offset;
  
  os_loadinit.close();
  
}

void Actuator::DATA_reset() //
{
  for (int i = 0; i < NUMOFSLAVES; i++) //[i][0] = z^0, [i][1] = z^1.  2x1으로 1x1은 현재 값, 2x1은 이전 값인데 그값 초기화
  {
    Motor_pos = 0;
    Motor_vel = 0;
    Motor_torque = 0;
  }
}

void Actuator::DATA_Receive(input_GTWI_t** in_twitter_GTWI) //Motor_num은 class를 선언할 때 Ethercat 통신 순서대로 이미 정해줌
{

    //11-1-3.Data received from slave
    //Copy the data from received PDO
#ifdef SLAVE_GTWI
  
    position_raw = in_twitter_GTWI[Motor_Num]->TXPDO_ACTUAL_POSITION_DATA;
    velocity_raw = in_twitter_GTWI[Motor_Num]->TXPDO_ACTUAL_VELOCITY_DATA;
    torque_raw = in_twitter_GTWI[Motor_Num]->TXPDO_ACTUAL_TORQUE_DATA;
    //            Ain1_raw[i] = in_twitter_GTWI[i]->TXPDO_ANALOG_INPUT_1_DATA;
    //            DCvolt_raw[i] = in_twitter_GTWI[i]->TXPDO_DC_LINK_CIRCUIT_VOLTAGE_DATA;
    Din_raw = in_twitter_GTWI[Motor_Num]->TXPDO_DIGITAL_INPUTS_DATA;
    statusword = in_twitter_GTWI[Motor_Num]->TXPDO_STATUSWORD_DATA;
    modeofOP_disp = in_twitter_GTWI[Motor_Num]->TXPDO_MODE_OF_OPERATION_DISPLAY_DATA;
    
    
    DATA_unit_change();

#endif

#ifdef SLAVE_PTWI


#endif



}

void Actuator::DATA_Send(output_GTWI_t** out_twitter_GTWI) //
{

#ifdef SLAVE_GTWI
    // Have to check what the multipling factors are.
    out_twitter_GTWI[Motor_Num]->RXPDO_TARGET_POSITION_DATA = (int32)(target_position * (Enc_resolution / (2 * M_PI)));
    out_twitter_GTWI[Motor_Num]->RXPDO_TARGET_POSITION_DATA_0 = (int32)(target_position * (Enc_resolution / (2 * M_PI)));
    out_twitter_GTWI[Motor_Num]->RXPDO_TARGET_VELOCITY_DATA = (int32)(target_speed * (Enc_resolution / (2 * M_PI)));
    out_twitter_GTWI[Motor_Num]->RXPDO_TARGET_TORQUE_DATA = (int16)(target_torque * 1000000.0 / (45000 * Torque_constant * Gear_ratio)); // DS402 Maxon
    out_twitter_GTWI[Motor_Num]->RXPDO_DIGITAL_OUTPUTS_DATA = digital_output;
    out_twitter_GTWI[Motor_Num]->RXPDO_MAXIMAL_TORQUE = 1000; //?
    out_twitter_GTWI[Motor_Num]->RXPDO_CONTROLWORD_DATA = controlword;
    out_twitter_GTWI[Motor_Num]->RXPDO_CONTROLWORD_DATA_0 = controlword;
    out_twitter_GTWI[Motor_Num]->RXPDO_MODE_OF_OPERATION_DATA = modeOP;

#endif
}
// input current = 1000000/(45000*gear_ratio*torque_constant)
void Actuator::DATA_unit_change() {
  if(Motor_Num == 2||Motor_Num == 10||Motor_Num == 11)
  {
    if(Motor_Num == 11)
    Motor_pos = -(double)(position_raw / Enc_resolution * 2 * M_PI) /50;
    else 
    Motor_pos = -(double)(position_raw / Enc_resolution * 2 * M_PI) /100;
  }
  else
  {
    if(Motor_Num == 0 || Motor_Num == 5 || Motor_Num == 6)
    Motor_pos = (double)(position_raw / Enc_resolution * 2 * M_PI) /50;
    else 
    Motor_pos = (double)(position_raw / Enc_resolution * 2 * M_PI) /100;
//    if(Motor_Num == 1) 
//    cout<< "Knee position raw : "<< position_raw <<"\nhip motor_pos: "<< Motor_pos << "\nhip motor_jig_pos: " << Motor_jig_pos <<endl;
  } 
    if(Enc_init == true)
    {
      initial_enc_pos = Motor_pos;
      Motor_pos_offset = Motor_pos - Motor_jig_pos;
//      cout << Motor_pos_offset << endl;      
      QString LoggingPath = "/home/mcl/Documents/RT_ECAT_MASTER_ELMO_GOLD/initial_encoder/";
      LoggingPath.append(QString("Motor %1.txt").arg(Motor_Num));
      os_saveinit.open(LoggingPath.toStdString());
      os_saveinit<<std::left<<Motor_pos_offset;

      os_saveinit.close();
      
    } 
    
    Motor_pos = Motor_pos - Motor_pos_offset;
    
    if(Motor_Num == 1) 
    {
    cout << "Knee position raw : "<< position_raw <<"\nhip motor_pos: "<< Motor_pos << "\nhip motor_jig_pos: " << Motor_jig_pos <<endl;
    cout << "Motor_pos_offset: " << Motor_pos_offset << endl;
    }
    Motor_vel = (double)(velocity_raw / Enc_resolution * 2 * M_PI) /100;
    Motor_torque = (double)((double)torque_raw) * 45000 / 1000000;    
        
}

void Actuator::exchange_mutex() {
    if (!pthread_mutex_trylock(&data_mut)) {

    controlword = _M_CONTROLWORD[Motor_Num];
    modeOP = _M_MODE_OF_OPERATION[Motor_Num];
    target_torque = _M_motor_torque[Motor_Num];
    
    _M_motor_position[Motor_Num] = Motor_pos;
    _M_actual_current[Motor_Num] = Motor_torque;
    
    //cout << Motor_Num << ": " << Motor_pos << endl;
    
    //// Flag ////
    Enc_init = _M_Enc_init;
     
    pthread_mutex_unlock(&data_mut);
    }
}
