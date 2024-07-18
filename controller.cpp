#include "controller.h"

Controller::Controller()
{

}
//


double Controller::pid(Vector2d posRW_err, Vector2d posRW_err_old, int idx,int Leg_num, int mode)
                                       // posRW_err[0] : R direction
                                       // posRW_err[1] : th direction
                                       // posRW_err_old same
                                       // mode 0 : position control mode, mode 1 : velocity control, mode 2: Cascade?
{
  if(mode == 0) // mode = 0: position PID
  {
    kp = get_posPgain(Leg_num, idx);
    ki = get_posIgain(Leg_num, idx);
    kd = get_posDgain(Leg_num, idx);
    cutoff_freq = get_posD_cutoff(Leg_num, idx);
  }
  else if(mode == 1) // mode = 1: velocity PID
  {
    kp = get_velPgain(Leg_num, idx);
    ki = get_velIgain(Leg_num, idx);
    kd = get_velDgain(Leg_num, idx);
    cutoff_freq = get_velD_cutoff(Leg_num, idx);
  }
  else
  {
    kp = 0;
    ki = 0;
    kd = 0;
    cutoff_freq = 50;
  }
  double tau = 1 / (2 * PI * cutoff_freq);

  
  P_term[idx][0] = kp * posRW_err[idx];
  I_term[idx][0] = ki * T / 2 * (posRW_err[idx] + posRW_err_old[idx]) + I_term[idx][1];
  D_term[idx][0] = 2 * kd / (2 * tau + T) * (posRW_err[idx] - posRW_err_old[idx]) -
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

Vector2d controller::DOBRW(Vector2d DOB_output ,Matrix2d Lamda_nominal_DOB,double acc_m,double acc_b ,double cut_off ,int flag)
{
    //DOB_output이 한 step 이전 값이다. 그래서 여기 안에서 setting 안해줘도됨
    // old 값 initial 0으로 해줘야함
    // UI에 넣어야할 내용은 cut_off, flag


    double time_const = 1 / (2 * pi * cut_off); 

    // 정의는 여기서
    Vector2d result;

    Vector2d qddot;
    qddot[0] = acc_m;
    qddot[1] = acc_b;

    lhs_dob = DOB_output;
    rhs_dob = Lamda_nominal_DOB * qddot;
    
    // 현재값 계산
    for(int i = 0; i < 2; i++)
    {
      T_dob[i][0] = lhs_dob[i] - rhs_dob[i];
    }

    if (flag == true)
    {
      for(int i = 0; i < 2; i++)
      {
        tauDist_hat[i][0] = (2 * (T_dob[i][0] + T_dob[i][1]) - (Ts - 2 * time_const) * tauDist_hat[i][1]) / (Ts + 2 * time_const);
      }
    }
    else
    {
      for(int i = 0; i < 2; i++)
      {
        tauDist_hat[i][0] = 0;
      }
    }
    
    //old값 update
    for(int i = 0; i < 2; i++)
    {
      T_dob[i][1] = T_dob[i][0];
      tauDist_hat[i][1] = tauDist_hat[i][0];
    }

    result[0] = tauDist_hat[0][0];
    result[1] = tauDist_hat[1][0];
    

    return result;

}; // Rotating Workspace DOB

void controller::FOBRW(Vector2d DOB_output,Matrix2d Lamda_nominal_FOB,Matrix2d JacobianTrans,double acc_m,double acc_b ,double cut_off ,int flag)
{
    //DOB_output이 한 step 이전 값이다. 그래서 여기 안에서 setting 안해줘도됨
    // old 값 initial 0으로 해줘야함
    // UI에 넣어야할 내용은 cut_off, flag
    // Jacobian도 가져와야함
    
    
    double time_const = 1 / (2 * pi * cut_off);

    // 정의는 여기서
    Vector2d result;

    Vector2d qddot;
    qddot[0] = acc_m;
    qddot[1] = acc_b;

    lhs_dob = DOB_output;
    rhs_dob = Lamda_nominal_FOB * qddot;
    T_fob[0] = lhs_dob[0] - rhs_dob[0];
    
    // 현재값 계산
    if (flag == true)
    {
      tauExt_hat[0] = (2 * (T_fob[0] + T_fob[1]) - (Ts - 2 * time_const) * tauExt_hat[1]) / (Ts + 2 * time_const);
    }
    else
    {
      tauExt_hat[0] = 0;
    }

    result = JacobianTrans * tauExt_hat[0];
    forceExt_hat[0] = result[0]; // r direction
    
    //old값 update
    T_fob[1] = T_fob[0];
    tauExt_hat[1] = tauExt_hat[0];
    forceExt_hat[2] = forceExt_hat[1];
    forceExt_hat[1] = forceExt_hat[0];
    

}; // Rotating Workspace DOB

double controller::admittance(double omega_n, double zeta, double k)
{
  // admittance control
  // admittance control은 현재 사용하지 않음
  // 현재 omega_n, zeta, k 로 tunning 하고 있는데, 변환식을 통해 아래에 적어주면 된다
    double ad_M = k/(pow(omega_n,2));
    double ad_B = 2*zeta*k/omega_n;
    double ad_K = k;

    double c1 = 4 * ad_M + 2 * ad_B * Ts + ad_K * pow(Ts, 2);
    double c2 = -8 * ad_M + 2 * ad_K * pow(Ts, 2);
    double c3 = 4 * ad_M - 2 * ad_B * Ts + ad_K * pow(Ts, 2);
   
   deltaPos[0] =
        (pow(Ts, 2) * forceExt_hat[0] + 2 * pow(Ts, 2) * forceExt_hat[1] +
            pow(Ts, 2) * forceExt_hat[2] - c2 * deltaPos[1] - c3 * deltaPos[2]) / c1;


  return deltaPos[0];
}


/*-----------------------Initial function-------------------------*/
void controller::DOBinitial()
{ //Old값 초기화
  for(int i = 0; i < 2; i++)
  {
    T_dob[i][1] = 0;
    tauDist_hat[i][1] = 0;
  }
}

void controller::FOBinitial()
{ //Old값 초기화
  T_fob[1] = 0;
  tauExt_hat[1] = 0;
  forceExt_hat[1] = 0;
  forceExt_hat[2] = 0;
}

void controller::init()
{
  DOBinitial();
  FOBinitial();
}
