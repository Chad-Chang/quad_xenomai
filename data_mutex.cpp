#include<data_mutex.h>
#include<QTimer>

double _M_sampling_time_ms = 0; //sync(send) data from RT thread to GUI
int _M_overrun_cnt = 0;

uint16_t    _M_Ecat_states[NUMOFSLAVES+1+1];
int         _M_Ecat_WKC = 0;    //Actual Working Counter
int         _M_Ecat_expectedWKC = 0;    //Working Counter which is expected.

uint16_t    _M_STATUSWORD[NUMOFSLAVES];
int8_t      _M_MODE_OF_OPERATION_DISPLAY[NUMOFSLAVES];

int8_t     _M_MODE_OF_OPERATION[NUMOFSLAVES];
uint16_t    _M_CONTROLWORD[NUMOFSLAVES];

using std:: vector;

//// Parameter ////
double T = 0.001;


//// Motor parameter ////

double _M_motor_position[NUMOFSLAVES];
double _M_motor_acceleration[NUMOFSLAVES];
double _M_motor_torque[NUMOFSLAVES];
double _M_motor_velocity[NUMOFSLAVES];
double _M_ref_current[NUMOFSLAVES];

// actual current 
double _M_actual_current[NUMOFSLAVES];

//// Leg parameter (RW) ////
Vector4f _M_ref_r_pos;
Vector4f _M_ref_th_pos;
Vector4f _M_ref_r_vel;
Vector4f _M_ref_th_vel;

//// State in RW ////
Vector4f _M_RW_r_pos;
Vector4f _M_RW_th_pos;
Vector4f _M_RW_r_vel;
Vector4f _M_RW_th_vel;

//// RW error ////
Vector4f _M_r_pos_error;
Vector4f _M_th_pos_error;
Vector4f _M_r_vel_error;
Vector4f _M_th_vel_error;

//// Position PID Gain ////
Vector4f _M_RW_r_posIgain;
Vector4f _M_RW_r_posPgain;
Vector4f _M_RW_r_posDgain;
Vector4f _M_RW_r_posD_cutoff;

Vector4f _M_RW_th_posIgain;
Vector4f _M_RW_th_posPgain;
Vector4f _M_RW_th_posDgain;
Vector4f _M_RW_th_posD_cutoff;

//// Velocity PID Gain ////
Vector4f _M_RW_r_velIgain;
Vector4f _M_RW_r_velPgain;
Vector4f _M_RW_r_velDgain;
Vector4f _M_RW_r_velD_cutoff;

Vector4f _M_RW_th_velIgain;
Vector4f _M_RW_th_velPgain;
Vector4f _M_RW_th_velDgain;
Vector4f _M_RW_th_velD_cutoff;

double _M_Homming_input[NUMOFSLAVES];
double _M_Motor_pos_init[NUMOFSLAVES];



//// Mutex connect ////
timespec data_mut_lock_timeout;
int timeout_ns;
QTimer *tim;

//////////// Flag ////////////
int _M_Traj_ON;
bool _M_Ctrl_on;
int _M_ctrl_mode;
bool _M_Enc_init;
bool _M_DOB_on;
bool _M_admittance_on;

bool _M_Homming_checked = false;
bool _M_Homming_clicked = false;
bool _M_DataLog_flag = false;
bool _M_stop;


