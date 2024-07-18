#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <data_mutex.h>
#include <actuator.h>
#include <filter.h>


class Controller
{
private:
  // PID //
  Vector4d RW_r_posPgain; // FL FR RL RR leg
  Vector4d RW_r_posIgain;
  Vector4d RW_r_posDgain;
  Vector4d RW_r_posD_cutoff;

  Vector4d RW_th_posPgain; // FL FR RL RR
  Vector4d RW_th_posIgain;
  Vector4d RW_th_posDgain;
  Vector4d RW_th_posD_cutoff;
  
  Vector4d RW_r_velPgain; // FL FR RL RR leg
  Vector4d RW_r_velIgain;
  Vector4d RW_r_velDgain;
  Vector4d RW_r_velD_cutoff;

  Vector4d RW_th_velPgain; // FL FR RL RR
  Vector4d RW_th_velIgain;
  Vector4d RW_th_velDgain;
  Vector4d RW_th_velD_cutoff;

  Vector2d PID_output;
  double cutoff_freq_pos = 150; 
  double cutoff_freq_vel = 150;
  double cutoff_freq = 150; 

  // Using in Function
  double P_term[2][2]; // first column is about r, second column is about theta
  double I_term[2][2];
  double D_term[2][2];
  double kp; double ki; double kd;
  double kp_pos; double ki_pos; double kd_pos;
  double kp_vel; double ki_vel; double kd_vel;
  Vector2d error;
  Vector2d error_old;

public:
  Controller();

  // Data set//
  void setDelayData();
  void Mutex_exchange();

  // feedback control //;
  double pid(Vector2d posRW_err, Vector2d posRW_err_old, Vector2d velRW_err, Vector2d velRW_err_old, int r0th1, int Leg_num, int mode); // idx:  r(=0), th(=1)중 어떤 state의 PD control?
  Vector2d velPID();                                                                 // Leg_num: FL-0 FR-1 RL-2 RR-3

  double get_posPgain(int Leg_num, int r0th1);
  double get_posIgain(int Leg_num, int r0th1);
  double get_posDgain(int Leg_num, int r0th1);
  double get_posD_cutoff(int Leg_num, int r0th1);
  
  double get_velPgain(int Leg_num, int r0th1);
  double get_velIgain(int Leg_num, int r0th1);
  double get_velDgain(int Leg_num, int r0th1);
  double get_velD_cutoff(int Leg_num, int r0th1);
};

#endif // CONTROLLER_H
