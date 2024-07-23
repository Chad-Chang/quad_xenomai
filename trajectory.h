#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <data_mutex.h>
#include <controller.h>

class trajectory
{
private:
  Controller joint_c[NUMOFSLAVES]; // joint controller
  bool Homming_checked = false;
  bool Homming_clicked = false;
  int Homming_t = 0;
  int pos_traj_t = 0;
  int vel_traj_t = 0;
  double Homming_duration = 2.5;
  double Homming_gain = 120;
  double Motor_Home_pos[14];
  double Motor_pos_init[14];
  double Motor_pos[14];
  double Homming_input[14];
  double joint_kp[NUMOFSLAVES]= {150,200,150,  250,250,150, 150,200,200,  200,200,150,   0,0};
  double joint_kd[NUMOFSLAVES]= {2,2.5,2,      2,2,2,       2,2,2,        2,2,2,      0,0};
  double joint_ki[NUMOFSLAVES]= {0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  double Homming_traj[NUMOFSLAVES];
  
  // SLIP profile
  Vector2f SLIP_pos_profile;
  Vector2f SLIP_vel_profile; 
  
  
  
public:
  trajectory();
  double* homming();
  void exchange_mutex();
  
  // test trajecotry code
  Vector2f test_rw_pos_cos_traj(MatrixXf rw_pos, MatrixXf rw_pos_curr, int time_interval); 
  Vector2f test_rw_vel_cos_traj(MatrixXf rw_vel, MatrixXf rw_vel_curr, int time_interval);
  
  Vector2f SLIP_vel_traj(MatrixXf rw_vel, MatrixXf rw_vel_curr, int time_interval); // velocity profile, position profile include 
   
  
};

#endif // TRAJECTORY_H
// motor[0] = 60
// motor[1] = 180
// motor[2] = 190
// motor[3] = 

