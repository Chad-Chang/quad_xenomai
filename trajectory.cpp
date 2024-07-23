#include "trajectory.h"

trajectory::trajectory()
{
for(int i = 0 ; i<NUMOFSLAVES  ; i++ )
  {
    joint_c[i].j_set_gain(joint_kp[i],joint_ki[i],joint_kd[i]);
  }
  
for(int i = 0; i < NUMOFSLAVES; i ++)
  {Homming_input[i] = 0;}
  
    //*************** Home Position ***************//
    Motor_Home_pos[0] = 0;
    Motor_Home_pos[1] = 0.7854; //HIP
    
    Motor_Home_pos[2] = 2.3562; //KNEE
    Motor_Home_pos[3] = 2.3562; //KNEE
    Motor_Home_pos[4] = 0.7854; //HIP
    Motor_Home_pos[5] = 0;
    Motor_Home_pos[6] = 0;
    Motor_Home_pos[7] = 0.7854;
    Motor_Home_pos[8] = 2.3562;
    Motor_Home_pos[9] = 2.3562;
    Motor_Home_pos[10] = 0.7854;
    Motor_Home_pos[11] = 0;
    Motor_Home_pos[12] = 0;
    Motor_Home_pos[13] = 0;
    cout << " ========= Trajectory Constructor ! =========" << endl;
}

double* trajectory::homming()
{  
for(int i = 0 ; i<NUMOFSLAVES  ; i++ )
  {
   joint_c[i].j_setDelayData();
  }
    if(Homming_checked == true)
      {
        Homming_t++;
        if(Homming_t*0.001 < Homming_duration)
        { 
          for(int i = 0 ; i<NUMOFSLAVES; i++ )
          {
          
            Homming_traj[i] =  0.5*(Motor_Home_pos[i]-Motor_pos_init[i])*(1-cos(PI/Homming_duration*(Homming_t*0.001)))+Motor_pos_init[i];
            Homming_input[i] = joint_c[i].j_posPID(Homming_traj[i],Motor_pos[i],T,75);
          }
        }
        else
        {
          for (int i = 0; i < NUMOFSLAVES; i ++)
            Homming_input[i] = joint_c[i].j_posPID(Motor_Home_pos[i],Motor_pos[i],T,75);  
        }
      }
  else{
        Homming_t = 0;
        for (int i = 0; i < NUMOFSLAVES; i ++) // initialize joint PID data for tunning.
          joint_c[i].j_reset();
        }
  
  return Homming_input;
}

void trajectory::exchange_mutex()
{

  //// flag
  Homming_checked = _M_Homming_checked;
  Homming_clicked = _M_Homming_clicked;  
          
  for(int i = 0; i < NUMOFSLAVES; i ++)
  {  
    Motor_pos_init[i] = _M_Motor_pos_init[i];
    Motor_pos[i] = _M_motor_position[i];
  }        
} 

Vector2f trajectory::test_rw_pos_cos_traj(MatrixXf rw_pos, MatrixXf rw_pos_curr, int time_interval)
{
  pos_traj_t ++; 
  Vector2f rw_pos_trajectory;
  rw_pos_trajectory << 0,0;
  
  if(pos_traj_t*0.001 <=time_interval)
  {
    for(int i = 0 ; i< 4; i++)
    {
      rw_pos_trajectory.col(i) = 0.5*(rw_pos.col(i)-rw_pos_curr.col(i))*(1-cos((M_PI/time_interval)*pos_traj_t*0.001))+rw_pos_curr.col(i);
    }
  }
  else
  {
    cout << " ===================== pos trajectory end ========================="<< endl;
    rw_pos_trajectory =  rw_pos_curr;
    pos_traj_t = 0;
  }
  return rw_pos_trajectory;
} 


Vector2f trajectory::test_rw_vel_cos_traj(MatrixXf rw_vel, MatrixXf r_vel_curr, int time_interval)
{
  vel_traj_t ++; 
  Vector2f rw_vel_trajectory(2,4);
  rw_vel_trajectory << 0,0,0,0,
                       0,0,0,0;
  
  if(vel_traj_t*0.001 <=time_interval)
  {
    for(int i = 0 ; i< 2; i++)
    {
      rw_vel_trajectory.col(i) = 0.5*(rw_vel.col(i)-r_vel_curr.col(i))*(1-cos((M_PI/time_interval)*vel_traj_t*0.001))+r_vel_curr.col(i);
    }
  }
  else
  {
    cout << " ===================== vel trajectory end ========================="<< endl;
    rw_vel_trajectory <<0,0; // stop
    vel_traj_t = 0;
  }
  return rw_vel_trajectory;
}
