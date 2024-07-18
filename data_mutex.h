#ifndef DATA_MUTEX_H
#define DATA_MUTEX_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <QTimer>
#include <cstdint>
#include <inttypes.h>
#include <pthread.h>
#include <qcustomplot.h>>
#include <stdio.h>
#include <unistd.h>
#include <vector>
#include <iostream>

using namespace Eigen;
using std::vector;
using namespace std;

#define NUMOFSLAVES 12 // Number of elmo slaves
#define NUMOF_GTWI_SLAVES 12
#define NUMOF_PTWI_SLAVES 0
#define PI	(3.14159265359)
#define NUMOFLEGS 4

//// MUTEX TOOLS ////
extern pthread_mutex_t data_mut;
extern timespec data_mut_lock_timeout;
extern int timeout_ns;
extern QTimer *tim;

extern double T;

//// ELMO ////
extern double _M_sampling_time_ms;
extern int _M_overrun_cnt;
extern int _M_Ecat_WKC; //
extern int _M_Ecat_expectedWKC;

extern uint16_t _M_STATUSWORD[NUMOFSLAVES];
extern int8_t _M_MODE_OF_OPERATION_DISPLAY[NUMOFSLAVES];

extern int8_t _M_MODE_OF_OPERATION[NUMOFSLAVES];
extern uint16_t _M_CONTROLWORD[NUMOFSLAVES];

//// Motor parameter ////
extern double _M_motor_position[NUMOFSLAVES];
extern double _M_motor_torque[NUMOFSLAVES];
extern double _M_motor_velocity[NUMOFSLAVES];
extern double _M_ref_current[NUMOFSLAVES];


//// Leg parameter (RW) ////
extern Vector4d _M_ref_r_pos;
extern Vector4d _M_ref_th_pos;
extern Vector4d _M_ref_r_vel;
extern Vector4d _M_ref_th_vel;

//// State in RW ////
extern Vector4d _M_RW_r_pos;
extern Vector4d _M_RW_th_pos;
extern Vector4d _M_RW_r_vel;
extern Vector4d _M_RW_th_vel;

//// RW error ////
extern Vector4d _M_r_pos_error;
extern Vector4d _M_th_pos_error;
extern Vector4d _M_r_vel_error;
extern Vector4d _M_th_vel_error;

//// Position PID Gain ////
extern Vector4d _M_RW_r_posIgain;
extern Vector4d _M_RW_r_posPgain;
extern Vector4d _M_RW_r_posDgain;
extern Vector4d _M_RW_r_posD_cutoff;

extern Vector4d _M_RW_th_posIgain;
extern Vector4d _M_RW_th_posPgain;
extern Vector4d _M_RW_th_posDgain;
extern Vector4d _M_RW_th_posD_cutoff;

//// Velocity PID Gain ////
extern Vector4d _M_RW_r_velIgain;
extern Vector4d _M_RW_r_velPgain;
extern Vector4d _M_RW_r_velDgain;
extern Vector4d _M_RW_r_velD_cutoff;

extern Vector4d _M_RW_th_velIgain;
extern Vector4d _M_RW_th_velPgain;
extern Vector4d _M_RW_th_velDgain;
extern Vector4d _M_RW_th_velD_cutoff;

extern double _M_actual_current[NUMOFSLAVES];

//// Flag ////

extern int _M_traj_ON;
extern bool _M_Enc_init;
extern int _M_ctrl_mode; // control mode

#endif // DATA_MUTEX_H
