#ifndef KINEMATICS_H
#define KINEMATICS_H

#include<data_mutex.h>
#include<actuator.h>  

class Kinematics
{

private:
    Vector2d posRW; // r, th 순서의 2x1벡터
    Vector2d velRW; // r, th 순서의 2x1벡터
    
    double posRW_error[2][2];
    double velRW_error[2][2];
    
    //******************
    double r_pos_error[4];// 
    double th_pos_error[4];
    double r_vel_error[4];// 
    double th_vel_error[4];
    
    double r_posRW[4];
    double th_posRW[4];
    double r_velRW[4];
    double th_velRW[4];

    double ref_r_pos[4];
    double ref_th_pos[4];
    double ref_r_vel[4];
    double ref_th_vel[4];

    //******************
    /*Trunk Parameter*/
    double m_hip;         // mass of hip torso
    double m_trunk_front; // mass of front trunk
    double m_trunk_rear;  // mass of rear trunk
    double m_trunk;       // total mass of trunk
    double m_total;       // total robot mass

    /* Leg Parameter */  
    double L; // leg length : thigh and shank links' length are assumed to be the same

    double m_thigh; // mass of thigh link
    double m_shank; // mass of shank link
    double m_leg;   // mass of leg

    double d_thigh; // CoM pos of thigh w.r.t HFE
    double d_shank; // CoM pos of shank w.r.t KFE

    double Izz_thigh; // MoI(z) of thigh w.r.t its CoM
    double Izz_shank; // MoI(z) of shank w.r.t its CoM

    double Jzz_thigh; // MoI(z) of thigh w.r.t HFE
    double Jzz_shank; // MoI(z) of shank w.r.t KFE

    double JzzR_thigh;
    double JzzR_shank;
    double JzzR_couple;

    Matrix2d MatInertia_bi;
    Matrix2d MatInertia_RW;
    Matrix2d Inertia_DOB;
    Matrix2d Lamda_nominal_FOB;
    Matrix2d Lamda_nominal_DOB;
    
    //******************
    Matrix2d Jacobian;
    Matrix2d JacobianTrans;
    
    Vector2d veljoint;
  public:
    Kinematics();
  
    void pos_trajectory(int traj_t, int Leg_num);
    void vel_trajectory(int traj_t, int Leg_num);
    void Cal_RW(double thm, double thb, double thmdot, double thbdot, int Leg_num);
    void model_param_cal(double thm,double thb);

    void set_DelayDATA();
    Matrix2d RW_Jacobian(){ return Jacobian; };
    Vector2d get_posRW() { return posRW; };
    Vector2d get_posRW_error(int idx);
    Vector2d get_velRW() {return velRW; };
    Vector2d get_velRW_error(int idx);
    
    Matrix2d get_RW_Jacobian() { return Jacobian; };
    Matrix2d get_RW_Jacobian_Trans() { return JacobianTrans; };
    Matrix2d get_Lamda_nominal_FOB() { return Lamda_nominal_FOB; };
    Matrix2d get_Lamda_nominal_DOB() { return Lamda_nominal_DOB; };
//    void exchange_mutex(); 
    void exchange_mutex(int Leg_num);
};

#endif // KINEMATICS_H
