#include "trajectory.h"

trajectory::trajectory()
{
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
//cout << Motor_pos_init[1] << endl;
    if(Homming_checked == true)
      { 
        Homming_t++;
        if(Homming_t*0.001 < Homming_duration)
        {
          for (int i = 0; i < NUMOFSLAVES; i ++)
            {Homming_input[i] = Homming_gain * (0.5*(Motor_Home_pos[i]-Motor_pos_init[i])*(1-cos(PI/Homming_duration*(Homming_t*0.001)))+Motor_pos_init[i]-Motor_pos[i]) ;  }
//          cout << "traj: " << 0.5*(Motor_Home_pos[1]-Motor_pos_init[1])*(1-cos(PI/Homming_duration*(Homming_t*0.001))) << "\n Motor_pos[1]: " << Motor_pos[1] << endl;  
//          cout << "traj: " << 0.5*(Motor_Home_pos[2]-Motor_pos_init[2])*(1-cos(PI/Homming_duration*(Homming_t*0.001))) << "\n Motor_pos[2]: " << Motor_pos[2] << endl;  

        }
        else
        {
          for (int i = 0; i < NUMOFSLAVES; i ++)
            Homming_input[i] = Homming_gain * (Motor_Home_pos[i]-Motor_pos[i]);  
            
        }
      }
  else{Homming_t = 0;}
  
  return Homming_input;
}

void trajectory::exchagne_mutex()
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

