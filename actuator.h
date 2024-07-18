#ifndef ACTUATOR_H
#define ACTUATOR_H
#include<cstdint>
#include<data_mutex.h>
#include <ethercat.h>
#include <ecat_func.h>
#include <cmath>

#define Torque_constant 0.035
#define Gear_ratio 100
class Actuator
{
private:
  // PDO mapped

  // RECEIVE
  int32_t position_raw{0}; // Raw data는 ELMO에서 받아오는 데이터를 그대로 받아오는 것
  int32_t velocity_raw{0};
  int16_t torque_raw{0};  
  uint32_t Din_raw{0};
  uint16_t statusword{0};
  int8_t modeofOP_disp{0};

  // SEND                                          //Define and initialization RxPDO buffers
  int8_t modeOP{0};
  uint16_t controlword{0};
  double target_torque{0};
  double target_speed{0};
  double target_position{0};
  uint32_t digital_output{0};
  
  // Initial position //
  double Motor_initial_pos;
  double Motor_pos_offset = 0;
  
  // ////////////// DATA //////////////////////
  double Motor_pos[2]; // old값 setting 할려고 두개로 만듬
  double Motor_vel[2]; // old값 초기화 해줘야함
  double Motor_acc[2];
  double Motor_torque;


  // ///////////// Which motor ///////////////
  int Motor_Num;
  

  // /////////// Caculated ///////////////////

  double Enc_resolution = 20000.0;
  double Pterm[NUMOFSLAVES][3]{0,};
  double Iterm[NUMOFSLAVES][3]{0,};
  double Dterm[NUMOFSLAVES][3]{0,};
  
  //// Flag ////
  bool Enc_init = 0;


public:
  Actuator(int Motor_num, double motor_init_pos);
  void DATA_Receive(input_GTWI_t **in_twitter_GTWI);
  void DATA_Send(output_GTWI_t **out_twitter_GTWI);
  void DATA_unit_change();
  void acc_cal(double tau);
  void setDelayData();

  void exchange_mutex();
  double getMotor_pos() { return Motor_pos[0]; }; // return first address of Motor_pos
  double getMotor_vel() { return Motor_vel[0]; };
  double getMotor_acc() { return Motor_acc[0]; };
  double getMotor_torque() { return Motor_torque; };
};

#endif // ACTUATOR_H
