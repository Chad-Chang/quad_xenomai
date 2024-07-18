#include "controller.h"

Controller::Controller()
{

}
//


double Controller::pid(Vector2d posRW_err, Vector2d posRW_err_old, Vector2d velRW_err, Vector2d velRW_err_old, int idx,int Leg_num, int mode)
                                       // posRW_err[0] : R direction
                                       // posRW_err[1] : th direction
                                       // posRW_err_old same
                                       // mode 0 : position control mode, mode 1 : velocity control, mode 2: Cascade?
{
    kp_pos = get_posPgain(Leg_num, idx);
    ki_pos = get_posIgain(Leg_num, idx);
    kd_pos = get_posDgain(Leg_num, idx);
    cutoff_freq_pos = get_posD_cutoff(Leg_num, idx);
    kp_vel = get_velPgain(Leg_num, idx);
    ki_vel = get_velIgain(Leg_num, idx);
    kd_vel = get_velDgain(Leg_num, idx);
    cutoff_freq_vel = get_velD_cutoff(Leg_num, idx);
  if(mode == 0) // mode = 0: position PID
  {
    kp = kp_pos;
    ki = ki_pos;
    kd = kd_pos;
    cutoff_freq = cutoff_freq_pos;
    for (int i = 0; i < 2; i++)
    {error[i] = posRW_err[i];
     error_old[i] =  posRW_err_old[i];}
  }
  else if(mode == 1) // mode = 1: velocity PID
  {
    kp = kp_vel;
    ki = ki_vel;
    kd = kd_vel;
    cutoff_freq = cutoff_freq_vel;
        for (int i = 0; i < 2; i++)
    {error[i] = velRW_err[i];
     error_old[i] =  velRW_err_old[i];}
  }
  else
  {
    kp = 0;
    ki = 0;
    kd = 0;
    cutoff_freq = 50;
  }
  double tau = 1 / (2 * PI * cutoff_freq);

  
  P_term[idx][0] = kp * error[idx];
  I_term[idx][0] = ki * T / 2 * (error[idx] + error_old[idx]) + I_term[idx][1];
  D_term[idx][0] = 2 * kd / (2 * tau + T) * (error[idx] - error_old[idx]) -
                       (T - 2 * tau) / (2 * tau + T) * D_term[idx][1]; // 이 함수 내에서 r_err는 주소값 but [0]와 같은 배열 위치로 원소를
                                                                           // 특정해주면 그 부분의 value가 된다.(이건 그냥 c++ 문법)
  PID_output[idx] = P_term[idx][0] + D_term[idx][0] + I_term[idx][0];
  
  return PID_output[idx];
  //
  setDelayData();
}

void Controller::setDelayData() {
  for (int i = 0; i < 2; i++) //[i][0] = z^0, [i][1] = z^1 ->  delay data 만들어 주는 function
  {
    /****************** Delay Data ******************/
    // r
    D_term[i + 1][0] = D_term[i][0];
    I_term[i + 1][0] = I_term[i][0];
    P_term[i + 1][0] = P_term[i][0];

    // th
    D_term[i + 1][1] = D_term[i][1];
    I_term[i + 1][1] = I_term[i][1];
    P_term[i + 1][1] = P_term[i][1];
  }
}

void Controller::Mutex_exchange() // Mutex에서 데이터를 받아오는 function.  Ui gain setting -> Mutex -> 여기 순서대로 이동 된 값임
{
    // RECEIVE
    if (!pthread_mutex_trylock(&data_mut)) {

        for (int i = 0; i < 4; i++) 
        { // 
          RW_r_posPgain[i] = _M_RW_r_posPgain[i];
          RW_r_posIgain[i] = _M_RW_r_posIgain[i];
          RW_r_posDgain[i] = _M_RW_r_posDgain[i];
          RW_r_posD_cutoff[i] = _M_RW_r_posD_cutoff[i];
          RW_th_posPgain[i] = _M_RW_th_posPgain[i];
          RW_th_posIgain[i] = _M_RW_th_posIgain[i];
          RW_th_posDgain[i] = _M_RW_th_posDgain[i];
          RW_th_posD_cutoff[i] = _M_RW_th_posD_cutoff[i];
        }
        pthread_mutex_unlock(&data_mut);
    }

    // SEND
}

double Controller::get_posPgain(int Leg_num, int r0th1) {
    if (r0th1 == 0)
      return RW_r_posPgain[Leg_num];
    else
      return RW_th_posPgain[Leg_num];
  };

double Controller:: get_posIgain(int Leg_num, int r0th1) {
    if (r0th1 == 0)
      return RW_r_posIgain[Leg_num];
    else
      return RW_th_posIgain[Leg_num];
  };

double Controller::get_posDgain(int Leg_num, int r0th1) {
    if (r0th1 == 0)
      return RW_r_posDgain[Leg_num];
    else
      return RW_th_posDgain[Leg_num];
  };
double Controller::get_posD_cutoff(int Leg_num, int r0th1) {
    if (r0th1 == 0)
      return RW_r_posD_cutoff[Leg_num];
    else
      return RW_th_posD_cutoff[Leg_num];
  };
  
  
// ==================================velocity gain callback=============================================
double Controller::get_velPgain(int Leg_num, int r0th1) {
    if (r0th1 == 0)
      return RW_r_velPgain[Leg_num];
    else
      return RW_th_velPgain[Leg_num];
};

double Controller:: get_velIgain(int Leg_num, int r0th1) {
  if (r0th1 == 0)
    return RW_r_velIgain[Leg_num];
  else
    return RW_th_velIgain[Leg_num];
};

double Controller::get_velDgain(int Leg_num, int r0th1) {
  if (r0th1 == 0)
    return RW_r_velDgain[Leg_num];
  else
    return RW_th_velDgain[Leg_num];
};
double Controller::get_velD_cutoff(int Leg_num, int r0th1) {
  if (r0th1 == 0)
    return RW_r_velD_cutoff[Leg_num];
  else
    return RW_th_velD_cutoff[Leg_num];
};
  
