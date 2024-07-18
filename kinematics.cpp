#include "kinematics.h"

Kinematics::Kinematics() {

}

void Kinematics::Cal_RW(double thm, double thb, double thmdot, double thbdot, int Leg_num)
{

  double th2 = thb - thm;
  
  veljoint[0] = thmdot;
  veljoint[1] = thbdot;
  
//  cout << th2 << endl;
  
  Jacobian(0, 0) = sin(th2 / 2);
  Jacobian(0, 1) = -sin(th2 / 2);
  Jacobian(1, 0) = cos(th2 / 2);
  Jacobian(1, 1) = cos(th2 / 2);

  Jacobian = L * Jacobian;
  JacobianTrans = Jacobian.transpose();
  
  posRW[0] = 2 * L * cos((thb - thm) / 2);
  posRW[1] = 0.5 * (thm + thb);
  
  velRW = Jacobian*veljoint;  
  
  r_posRW[Leg_num] = posRW[0];
  th_posRW[Leg_num] = posRW[1];

  //cout << "r_posRW_RL: " << r_posRW[Leg_num] << endl; 

}

void Kinematics::set_DelayDATA() {
  for (int i = 0; i < 2; i++) //[i][0] = z^0, [i][1] = z^1 ...
  {

    // Delay data
    posRW_error[i][1] = posRW_error[i][0];
    velRW_error[i][1] = velRW_error[i][0];
  }
}


void Kinematics::exchange_mutex(int Leg_num) {
  if (!pthread_mutex_trylock(&data_mut)) 
  {
    _M_ref_r_pos[Leg_num] = ref_r_pos[Leg_num];
    _M_ref_th_pos[Leg_num] = ref_th_pos[Leg_num];
    _M_ref_r_vel[Leg_num] = ref_r_vel[Leg_num];
    _M_ref_th_vel[Leg_num] = ref_th_vel[Leg_num];
    
    _M_RW_r_pos[Leg_num] = r_posRW[Leg_num]; 
    _M_RW_th_pos[Leg_num] = th_posRW[Leg_num];
    _M_RW_r_vel[Leg_num] = r_velRW[Leg_num]; 
    _M_RW_th_vel[Leg_num] = th_velRW[Leg_num];
    
    _M_r_pos_error[Leg_num] = r_pos_error[Leg_num]; // r direction error 
    _M_th_pos_error[Leg_num] = th_pos_error[Leg_num];
    _M_r_vel_error[Leg_num] = r_vel_error[Leg_num]; // r direction error 
    _M_th_vel_error[Leg_num] = th_vel_error[Leg_num];
    
    
    
    pthread_mutex_unlock(&data_mut); 
  }
}



Vector2d Kinematics ::get_posRW_error(int idx) // idx = 0이면 현재 값, idx = 1이면 이전 값
{

  Vector2d RWpos_error;
  Vector2d RWpos_error_old;

  if (idx == 0) {
    RWpos_error[0] = posRW_error[0][0];
    RWpos_error[1] = posRW_error[1][0];
  
  
//    cout << " RWpos_err_0_2" <<RWpos_error[0] << endl;
//    cout << " RWpos_err_1_2" <<RWpos_error[1] << endl;
    
    return RWpos_error;
  } else {
    RWpos_error_old[0] = posRW_error[0][1];
    RWpos_error_old[1] = posRW_error[1][1];

    return RWpos_error_old;
  }
}

Vector2d Kinematics ::get_velRW_error(int idx) // idx = 0이면 현재 값, idx = 1이면 이전 값
{

  Vector2d RWvel_error;
  Vector2d RWvel_error_old;

  if (idx == 0) {
    RWvel_error[0] = velRW_error[0][0];
    RWvel_error[1] = velRW_error[1][0];
  
  
//    cout << " RWpos_err_0_2" <<RWpos_error[0] << endl;
//    cout << " RWpos_err_1_2" <<RWpos_error[1] << endl;
    
    return RWvel_error;
  } else {
    RWvel_error_old[0] = velRW_error[0][1];
    RWvel_error_old[1] = velRW_error[1][1];

    return RWvel_error_old;
  }
}


void Kinematics::pos_trajectory(int traj_t, int Leg_num)
{

  double f = 0.5;
  
  //Leg_Num = FL(0), FR(1), RL(2), RR(3)
  switch(Leg_num)
  {
    case 0: // FL position trajectory

      ref_r_pos[0] = 0.05*sin(2*PI*f*0.001*traj_t) + 0.25;
      ref_th_pos[0] = PI/2;
      
      posRW_error[0][0] = ref_r_pos[0] - posRW[0];
      posRW_error[1][0] = ref_th_pos[0] - posRW[1];

      r_pos_error[0] = posRW_error[0][0];
      th_pos_error[0] = posRW_error[1][0];

  
    case 1: // FR position trajectory
      ref_r_pos[1] = 0.05*sin(2*PI*f*0.001*traj_t) + 0.25;
      ref_th_pos[1] = PI/2;
      
      posRW_error[0][0] = ref_r_pos[1] - posRW[0];
      posRW_error[1][0] = ref_th_pos[1] - posRW[1];
      
      r_pos_error[1] = posRW_error[0][0];
      th_pos_error[1] = posRW_error[1][0];
      
    case 2: // RL position trajectory
      ref_r_pos[2] =0.05*sin(2*PI*f*0.001*traj_t) + 0.25;
      ref_th_pos[2] = PI/2;
      
      posRW_error[0][0] = ref_r_pos[2] - posRW[0];
      posRW_error[1][0] = ref_th_pos[2] - posRW[1];
      
      r_pos_error[2] = posRW_error[0][0];
      th_pos_error[2] = posRW_error[1][0];

      
    case 3: // RR position trajectory
      ref_r_pos[3] =0.05*sin(2*PI*f*0.001*traj_t) + 0.25;
      ref_th_pos[3] = PI/2;
      
      posRW_error[0][0] = ref_r_pos[3] - posRW[0];
      posRW_error[1][0] = ref_th_pos[3] - posRW[1];
      
      
      r_pos_error[3] = posRW_error[0][0];
      th_pos_error[3] = posRW_error[1][0];
      
  }
}

void Kinematics::vel_trajectory(int traj_t, int Leg_num)
{

  double f = 0.5;
  
  //Leg_Num = FL(0), FR(1), RL(2), RR(3)
  switch(Leg_num)
  {
    case 0: 
    // FL position trajectory
      ref_r_vel[0] = 0.05*sin(2*PI*f*0.001*traj_t) + 0.25;
      ref_th_vel[0] = PI/2;
      
    // Error
      velRW_error[0][0] = ref_r_vel[0] - velRW[0];
      velRW_error[1][0] = ref_th_vel[0] - velRW[1];
      r_vel_error[0] = velRW_error[0][0];
      th_vel_error[0] = velRW_error[1][0];

  
    case 1: 
    // FR position trajectory
      ref_r_vel[1] = 0.05*sin(2*PI*f*0.001*traj_t) + 0.25;
      ref_th_vel[1] = PI/2;
      
    // Error
      velRW_error[0][0] = ref_r_vel[1] - velRW[0];
      velRW_error[1][0] = ref_th_vel[1] - velRW[1];
      r_vel_error[1] = velRW_error[0][0];
      th_vel_error[1] = velRW_error[1][0];
      
    case 2: 
    // RL position trajectory
      ref_r_vel[2] =0.05*sin(2*PI*f*0.001*traj_t) + 0.25;
      ref_th_vel[2] = PI/2;
    
    // Error
      velRW_error[0][0] = ref_r_vel[2] - velRW[0];
      velRW_error[1][0] = ref_th_vel[2] - velRW[1];
      r_vel_error[2] = velRW_error[0][0];
      th_vel_error[2] = velRW_error[1][0];

      
    case 3:
    // RR position trajectory
      ref_r_vel[3] =0.05*sin(2*PI*f*0.001*traj_t) + 0.25;
      ref_th_vel[3] = PI/2;
      
    // Error
      posRW_error[0][0] = ref_r_vel[3] - velRW[0];
      posRW_error[1][0] = ref_th_vel[3] - velRW[1];
      r_vel_error[3] = posRW_error[0][0];
      th_vel_error[3] = posRW_error[1][0];
      
  }
}

void Kinematics::model_param_cal(double thm,double thb)
{
  double th2 = thb - thm;
  /* Trunk Parameters */
    m_hip = 2.5;
    m_trunk_front = 10.;
    m_trunk_rear = 18.;
    m_trunk = 4 * m_hip + m_trunk_front + m_trunk_rear;

    /* Leg Parameters */
    L = 0.25;
    d_thigh = 0.11017; // local position of CoM of thigh
    d_shank = 0.12997; // local position of CoM of shank
    // printf("d_thigh : %f, d_shank : %f \n", d_thigh, d_shank);

    m_thigh = 1.017; // mass of thigh link
    m_shank = 0.143; // mass of shank link
    m_leg = m_thigh + m_shank;
    m_total = m_trunk + 4 * m_leg;
    // printf("m_thigh : %f, m_shank : %f \n", m_thigh, m_shank);

    Izz_thigh = 0.0057;     // MoI of thigh w.r.t. CoM
    Izz_shank = 8.0318e-04; // MoI of shank w.r.t. CoM
    // printf("Izz_thigh : %f, Izz_shank : %f \n", Izz_thigh, Izz_shank);

    Jzz_thigh =
        Izz_thigh + m_thigh * pow(d_thigh, 2); // MoI of thigh w.r.t. HFE
    Jzz_shank =
        Izz_shank + m_shank * pow(d_shank, 2); // MoI of thigh w.r.t. KFE
    // printf("Jzz_thigh : %f, Jzz_shank : %f \n", Jzz_thigh, Jzz_shank);

    double M1 = Jzz_thigh + m_shank * pow(L, 2);
    double M2 = m_shank * d_shank * L * cos(th2);
    double M12 = Jzz_shank;

    MatInertia_bi(0,0) = M1;
    MatInertia_bi(0,1) = M12;
    MatInertia_bi(1,0) = M12;
    MatInertia_bi(1,1) = M2;

    Lamda_nominal_FOB(0,0) = M1;
    Lamda_nominal_FOB(0,1) = M12;
    Lamda_nominal_FOB(1,0) = M12;
    Lamda_nominal_FOB(1,1) = M2;

    JzzR_thigh  = Jzz_thigh + Jzz_shank + m_shank * pow(L, 2) - 2 * m_shank * d_shank * L * cos(th2);
    JzzR_couple = Jzz_thigh + m_shank * pow(L, 2) - Jzz_shank;
    JzzR_shank = Jzz_thigh + Jzz_shank+ m_shank * pow(L, 2) + 2 * m_shank * d_shank * L * cos(th2);
    // printf("JzzR_thigh : %f, JzzR_shank : %f, JzzR_couple : %f \n", JzzR_thigh, JzzR_shank,
    // JzzR_couple);

    MatInertia_RW(0,0) = JzzR_thigh / (4 * pow(L, 2) * pow(sin(th2 / 2), 2));
    MatInertia_RW(0,1) = JzzR_couple / (2 * pow(L, 2) * sin(th2));
    MatInertia_RW(1,0) = JzzR_couple / (2 * pow(L, 2) * sin(th2));
    MatInertia_RW(1,1) = JzzR_shank / (4 * pow(L, 2) * pow(cos(th2 / 2), 2));
        
    
    Inertia_DOB(0,0) = MatInertia_RW(0,0);
    Inertia_DOB(0,1) = 0;
    Inertia_DOB(1,0) = 0;
    Inertia_DOB(1,1) = MatInertia_RW(1,1);

    
    
    Lamda_nominal_DOB = JacobianTrans*Inertia_DOB*Jacobian;
   
}