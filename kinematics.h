#ifndef KINEMATICS_H
#define KINEMATICS_H

#include<data_mutex.h>
#include<actuator.h>  

class Kinematics
{

private:
    double L = 0.25;
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
    Matrix2d Jacobian;
    Matrix2d JacobianTrans;
    
    Vector2d veljoint;
  public:
    Kinematics();
    void set_DelayDATA();
    Matrix2d RW_Jacobian(){ return Jacobian; };
    Vector2d get_posRW() { return posRW; };
    Vector2d get_posRW_error(int idx);
    Vector2d get_velRW() {return velRW; };
    Vector2d get_velRW_error(int idx);
    
    Matrix2d get_RW_Jacobian() { return Jacobian; };
    Matrix2d get_RW_Jacobian_Trans() { return JacobianTrans; };
//    void exchange_mutex(); 
    void exchange_mutex(int Leg_num);
    
    // delete
//    void RL_pos_trajectory(int &traj_t, bool isPressed, int Leg_num);
    void pos_trajectory(int traj_t, int Leg_num);
    void vel_trajectory(int traj_t, int Leg_num);
    void Cal_RW(double thm, double thb, double thmdot, double thbdot, int Leg_num);
    
};

#endif // KINEMATICS_H
