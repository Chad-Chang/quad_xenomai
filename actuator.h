#ifndef ACTUATOR_H
#define ACTUATOR_H
#include<cstdint>
#include<data_mutex.h>
#include <ethercat.h>
#include <ecat_func.h>
#include <cmath>
#include <fstream>

#define Torque_constant 0.035
#define Gear_ratio 100
class Actuator
{
private:
  // PDO mapped

  // RECEIVE
  int32_t position_raw{0};
  double additional_position_raw{0};
  uint32_t Auxiliary_position;
  int32_t velocity_raw{0};
  int16_t torque_raw{0};
  uint32 dc_link_circuit_voltage{0};
  int16_t Ain1_raw{0};
  uint32_t DCvolt_raw{0};
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
  double initial_enc_pos;
  double Motor_jig_pos;
  double Motor_pos_offset = 0;
  
  // ////////////// DATA //////////////////////
  double Motor_pos;
  double Motor_vel;
  double Motor_torque;

  double Motor_position;
  double Motor_velocity;

  // ///////////// Which motor ///////////////
  int Motor_Num;
  

  // /////////// Caculated ///////////////////

  double Enc_resolution = 20000.0;
  double Pterm[NUMOFSLAVES][3]{0,};
  double Iterm[NUMOFSLAVES][3]{0,};
  double Dterm[NUMOFSLAVES][3]{0,};
  
  //// Flag ////
  bool Enc_init = 0;
  
  //// init pos ////
  ofstream os_saveinit;
  ifstream os_loadinit;

public:
  Actuator(int Motor_num, double motor_jig_pos);
  void DATA_reset();
  void DATA_Receive(input_GTWI_t **in_twitter_GTWI);
  void DATA_Send(output_GTWI_t **out_twitter_GTWI);
  void DATA_unit_change();

  void exchange_mutex();
  double getMotor_pos() { return Motor_pos; }; // return first address of Motor_pos
  double getMotor_vel() { return Motor_vel; };
  double getMotor_torque() { return Motor_torque; };
};

#endif // ACTUATOR_H
