#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <data_mutex.h>
#include <actuator.h>
#include <filter.h>


class Controller
{
private:
  // PID //
  Vector4f RW_r_posPgain; // FL FR RL RR leg
  Vector4f RW_r_posIgain;
  Vector4f RW_r_posDgain;
  Vector4f RW_r_posD_cutoff;

  Vector4f RW_th_posPgain; // FL FR RL RR
  Vector4f RW_th_posIgain;
  Vector4f RW_th_posDgain;
  Vector4f RW_th_posD_cutoff;
  
  Vector4f RW_r_velPgain; // FL FR RL RR leg
  Vector4f RW_r_velIgain;
  Vector4f RW_r_velDgain;
  Vector4f RW_r_velD_cutoff;

  Vector4f RW_th_velPgain; // FL FR RL RR
  Vector4f RW_th_velIgain;
  Vector4f RW_th_velDgain;
  Vector4f RW_th_velD_cutoff;

  Vector2f PID_output;
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
  Vector2f error;
  Vector2f error_old;
  
   // joint PID
  double j_Kp_, j_Kd_, j_Ki_; 
  double j_I_term_[2] = {0};
  double j_D_term_[2] = {0};
  double j_P_term_ = 0;
  double j_imax_;
  double j_err[2] = {0};
  
 
  Vector2f rhs_dob; Vector2f lhs_dob;
  Matrix2f T_dob; // disturbance before Qfilter
  Matrix2f tauDist_hat; //disturbance after Qfilter 
  
  Vector2f rhs_fob; Vector2f lhs_fob;  
  Matrix2f T_fob; // temporal tau exter
  Matrix2f tauExt_hat; // matrix form 
  double forceExt_hat[3]; // r direction force_hat
  
  //admittance
  double deltaPos[3]; // r direction pose
  
public:
  Controller();

  // Data set//
  void setDelayData();
  void exchange_mutex();

  // feedback control //;
  double pid(Vector2f posRW_err, Vector2f posRW_err_old, Vector2f velRW_err, Vector2f velRW_err_old, int r0th1, int Leg_num, int mode); // idx:  r(=0), th(=1)중 어떤 state의 PD control?
  
  //joint PID//
  double jointpid(double joint_pos, double joint_pos_old);// Leg_num: FL-0 FR-1 RL-2 RR-3
  double Q_filter(double input, double input_old, double output_old, double cutoff_freq);             
  void j_set_gain(double Kp, double Ki, double Kd){j_Kp_ = Kp; j_Ki_ = Ki; j_Kd_ = Kd; }
  
  // reset PID input when reset
  void j_reset();
  double j_constraint(double v, double v_min, double v_max){return (v> v_max)?v_max : (v<v_min)?v_min:v;}
  // PID operator
  double j_posPID(double target, double current_ang, double dt, double cutoff);
  void j_setDelayData() {j_D_term_[1] = j_D_term_[0]; j_I_term_[1] = j_I_term_[0];}
  double j_get_posPgain() {return j_Kp_;}
  double j_get_posIgain() {return j_Ki_;}
  double j_get_posDgain() {return j_Kd_;}
  
  
  // DOB
  // Vector2f DOBRW(Vector2f DOB_output,Matrix2f Lamda_nominal_DOB, double acc_m, double acc_b, double cut_off, int flag);
  Vector2f DOBRW(Vector2f DOB_output,Matrix2f Lamda_nominal_DOB, Vector2f acc, double cut_off, int flag);
  void DOBinitial();
  
  // FOB  // flag : on/off
  void FOBRW(Vector2f DOB_output, Matrix2f Lamda_nominal_FOB, Matrix2f JacobianTrans, Vector2f acc ,double cut_off ,int flag); 
  void FOBinitial();
  
  //admittance
  double admittance(double omega_n,double zeta, double k); // r position return
  
  void init();

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
