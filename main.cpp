///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////* INCLUDE */////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
///
#include "mainwindow.h"
#include "controlwindow.h"

#include <QApplication>

///* General includes */
#include <errno.h>    //Header for defining macros for reporting and retrieving error conditions using the symbol 'errno'
#include <error.h>    //
#include <fcntl.h>    //C POSIX lib header. Header for opening and locking files and processing other tasks.
#include <inttypes.h> //
#include <iostream>
#include <malloc.h> //Memory allocation
#include <math.h>
#include <pthread.h>  //Header for using Thread operation from xenomai pthread.h
#include <rtdm/ipc.h> //
#include <signal.h>   //Header for signal processing
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>    //
#include <sys/timerfd.h> //
#include <unistd.h>      //C POSIX lib header. Header for accessing to the POSIX OS API

///* Personal includes *///
//Have to write includes in CMakeLists.txt in order to fully
//include personal headers.

#include <data_logging.h>
#include <actuator.h>
#include <controller.h>
#include <data_mutex.h>
#include <ecat_func.h>
#include <ethercat.h>
#include <filter.h>
#include <kinematics.h>
#include <trajectory.h>
#include <qcustomplot.h>


using namespace std;


#ifndef PI
#define PI	(3.14159265359)
#define PI2	(6.28318530718)
#endif

///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////* MUTEX VARIABLE *//////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

pthread_mutex_t data_mut = PTHREAD_MUTEX_INITIALIZER;


///////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////* SETUP FOR RT THREAD *///////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

#define RT_PERIOD_MS 1 //1msec
//#define CPU_AFFINITY_NUM 0 //Up to {$ grep processor /proc/cpuinfo | wc -l} - 1, 0~3 for Mini PC
#define XDDP_PORT 0 //0~{CONFIG-XENO_OPT_PIPE_NRDEV-1} //XENO_OPT_PIPE_NRDEV: No. of pipe devices

pthread_t rt;
int sigRTthreadKill = 0;

///////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////* DEFINITION FOR SOEM *///////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

const char *IFNAME = "eno1"; //Checked from SOEM simpletest //char*->const char*: After C++11,
char IOmap[4096]; //
int usedmem; //

int expectedWKC; //Working Counter:
volatile int wkc; //Doesn't want to occupy memory location

//Gold Twitter
//Mensioned in ecat_func.h as extern
//{No. of entries to be mapped, Entry addr., Entry addr., ...}
uint16 RXPDO_ADDR_GTWI[3] = {2, 0x1600, 0x1605}; //
//uint16 TXPDO_ADDR_GTWI[5] = {4, 0x1A02, 0x1A03, 0x1A18, 0x1A1D};
//uint16 TXPDO_ADDR_GTWI[6] = {5, 0x1A02, 0x1A03, 0x1A18, 0x1A1D, 0x1A1E};
uint16 TXPDO_ADDR_GTWI[3] = {2, 0x1A02,0x1A1E};

//Platinum Twitter
//uint16 RXPDO_ADDR_PTWI[2] = {1, 0x1600};
//uint16 TXPDO_ADDR_PTWI[2] = {1, 0x1A00};

//TS
uint16 TXPDO_ADDR_TS[4] = {3, 0x1A01, 0x1A02,0x1A03};


double offset = 0;
double knee_pos = 0;
double knee_pos_error = 0; 
///////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////* FUNCTION & VARIABLES DECLARATION */////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////


static void cleanup(void); //Delete, release of all handle, memory
static void *realtime_thread(void *arg); //RT thread

///////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////* Class declaration *//////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
MainWindow* w;
Controlwindow* c;

data_logging* Logging;

trajectory traj; // homming test
trajectory leg_traj[4]; // 0: FL, 1: FR, 2: RL, 3: RR
Kinematics leg_kinematics[4]; // 0: FL, 1: FR, 2: RL, 3: RR
Controller leg_controller[4]; // 0: FL, 1: FR, 2: RL, 3: RR
//  actuator configuration: HAA, HIP, KNEE

Actuator ACT_RLHAA(11, 0);
Actuator ACT_RLHIP(10, 0.546812);
Actuator ACT_RLKNEE(9, 2.59478);

Actuator ACT_RRHAA(6, 0);
Actuator ACT_RRHIP(7, 0.546812);  
Actuator ACT_RRKNEE(8, 2.59478);

Actuator ACT_FRHAA(5, 0);
Actuator ACT_FRHIP(4, 0.546812);
Actuator ACT_FRKNEE(3, 2.59478);

Actuator ACT_FLHAA(0, 0);
Actuator ACT_FLHIP(1, 0.546812); // 31.345degree
Actuator ACT_FLKNEE(2, 2.59478); // 148.67degree

// Jacobian
Matrix2f J_FL; Matrix2f J_FR;
Matrix2f J_RL; Matrix2f J_RR;

Matrix2f JT_FL; Matrix2f JT_FR;
Matrix2f JT_RL; Matrix2f JT_RR;

Matrix2f JT_inv_FL; Matrix2f JT_inv_FR;
Matrix2f JT_inv_RL; Matrix2f JT_inv_RR;

MatrixXf leg_motor_acc(2,4);  // hip -> knee 


// Controller output : RW PID output//
MatrixXf leg_RWpid_output(2,4); // 0: FL, 1: FR, 2: RL, 3: RR

// motor total control input _ tau_bi
MatrixXf leg_tau_bi_input(2,4); // 0: FL, 1: FR, 2: RL, 3: RR

Matrix2f DOB_Lambda_FL; Matrix2f DOB_Lambda_FR; 
Matrix2f DOB_Lambda_RL; Matrix2f DOB_Lambda_RR;

Matrix2f FOB_Lambda_FL; Matrix2f FOB_Lambda_FR; 
Matrix2f FOB_Lambda_RL; Matrix2f FOB_Lambda_RR;

double admit_pos[4] ;
// admittance 파라미터
double wn=20; double zeta=1; double k=30000; 

//========================================================== 

//========================================================== 

bool trajectory_test = 0;


///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////* Variable declaration *////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

double constant = 1000000.0 / (Torque_constant*Gear_ratio * 45000);
int t = 0;
/**************trajecotry time for each legs**************/
int traj_t = 0;

///////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////* Flag *//////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

bool Traj_on = false;// 0 before initialized -> temporal stop flag
bool Homming = false;
bool Ctrl_on = false;
bool stop = false;
// Homming

bool DOB_on = false;
bool admittance_on = false;
///////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////* Mode selcetion*//////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

int ctrl_mode = 0;

double* Homming_input;

// posRW
MatrixXf leg_RWpos(2,4); // 0: FL, 1: FR, 2: RL, 3: RR

MatrixXf leg_RWpos_err(2,4); // 0: FL, 1: FR, 2: RL, 3: RR

MatrixXf leg_RWpos_err_old(2,4); // 0: FL, 1: FR, 2: RL, 3: RR

// velRW
MatrixXf leg_RWvel(2,4);

MatrixXf leg_RWvel_err(2,4);

MatrixXf leg_RWvel_err_old(2,4);
Vector2d disturbance; 

MatrixXf leg_d_hat(2,4);
MatrixXf leg_r_force_hat(2,4);

// lhs_DOB 
MatrixXf DOB_taubi_lhs(2,4); // col : 0~4 = FL FR RL RR

// desired RW_pos => trajectory value
MatrixXf RW_pos_traj(2,4); MatrixXf RW_vel_traj(2,4); // row : r,w / col : Leg number
MatrixXf RW_pos_des(2,4); MatrixXf RW_vel_des(2,4); // row : r,w / col : Leg number

double traj_interval[4];

void param_initialize()
{   

    DOB_taubi_lhs << 0,0,0,0,
              0,0,0,0;
    cout <<"DOB lhs initialize"  <<endl;
    
    leg_RWpos_err_old << 0,0,0,0,
                      0,0,0,0;
    
    cout <<"RW pos error initialize"  <<endl;

    leg_RWvel_err_old << 0,0,0,0,
                      0,0,0,0;
    cout <<"RW vel error initialize"  <<endl;
    
}

///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////* MAIN FUNCTION *///////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char *argv[]) {
  mlockall(MCL_CURRENT | MCL_FUTURE); // Lock the memory page to prevent performance degrade
 
  pthread_attr_t rtattr, regattr; //
  sigset_t set;
  int sig;
  cpu_set_t cpus;
  int cpu_num = 0;

  sigemptyset(&set); //
  sigaddset(&set, SIGINT);
  sigaddset(&set, SIGTERM);
  sigaddset(&set, SIGHUP);
  pthread_sigmask(SIG_BLOCK, &set, NULL); // to send the system signal into the thread, not implemented in the thread yet.

  ////* THREAD SETTING *////
  struct sched_param p;
  int ret;

  ret = pthread_attr_init(&rtattr); // initialize pthread attribute
  if (ret)
    error(1, ret, "pthread_attr_init()");

  ret = pthread_attr_setinheritsched(&rtattr, PTHREAD_EXPLICIT_SCHED); // pthread scheduling inherit setting as explicit
  if (ret)
    error(1, ret, "pthread_attr_setinheritsched()");

  ret = pthread_attr_setschedpolicy(&rtattr, SCHED_FIFO); // pthread scheduling policy setting as FIFO
  if (ret)
    error(1, ret, "pthread_attr_setschedpolicy()");

  p.sched_priority = 99;
  ret = pthread_attr_setschedparam(&rtattr, &p); // setting scheduler parameter - priority 99 (Highest)
  if (ret)
    error(1, ret, "pthread_attr_setschedparam()");

  CPU_ZERO(&cpus);
  CPU_SET(cpu_num, &cpus);
  ret = pthread_attr_setaffinity_np(&rtattr, sizeof(cpus), &cpus); // give cpu affinity to be used to calculate for the RT thread
  if (ret)
    error(1, ret, "pthread_attr_setaffinity_np()");

  ret = pthread_create(&rt, &rtattr, realtime_thread, NULL); // create RT thread
  if (ret)
    error(1, ret, "pthread_create(realtime_thread)");

  pthread_attr_destroy(&rtattr); // delete pthread attribute union

  QApplication a(argc, argv);
  w = new MainWindow;
  c = new Controlwindow;
  Logging = new data_logging(c);
  
  
  w->show(); // main window show
  c->show(); // control window show
  param_initialize(); // error, ... parameter initialize

  ret = a.exec();      // Execute the mainwindow thread and return when GUI is terminated
  sigRTthreadKill = 1; // set(send) the kill signal to the RT thread

  usleep(2000); // wait time for exit the thread
  cleanup();    // clean up the thread

  return ret;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////* FUNCTION DEFINITION */////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

static void cleanup(void)
{
     pthread_cancel(rt); //thread end
     pthread_join(rt, NULL); //wait for thread is ended
}

static void *realtime_thread(void *arg)
{

#ifdef SLAVE_GTWI
     output_GTWI_t *out_twitter_GTWI[NUMOFSLAVES];    //RxPDO mapping data (output to the slaves)
     input_GTWI_t *in_twitter_GTWI[NUMOFSLAVES];      //TxPDO mapping data (input from the slaves)
#endif

///////////////////////////////////////* PARAMETER SETING */////////////////////////////////////

// EtherCAT
    bool ecatconfig_success = false;
    int chk;

// RT thread
    struct timespec trt;
    struct itimerspec timer_conf;
    struct timespec expected;

    long t1 = 0;
    long t2 = 0;
    long old_t1 = 0;
    long delta_t1 = 0;
    long t_before_sample_start = 0;
    double sampling_ms = 0;
    int tfd;

    uint32_t overrun = 0;
    uint64_t ticks;


///////////////////////////////////////* EherCAT MASTER */////////////////////////////////////

// 1. Initialize EtherCAT Master(Init)

    if(ecat_init(IFNAME)) //If there is any error, ecat_init returns '0'
        //There is no error with ecat_init
        sigRTthreadKill = 0;

    else
        //There is error
        sigRTthreadKill = 1;

// 2. EtherCAT slave number check

    #ifdef NON_SLAVE_TS // slave 없으면 thread 종료 
    if(ec_slavecount == NUMOFSLAVES)
    {
        sigRTthreadKill = 0;
        //for(int i=1;i<=NUMOFSLAVES-1;i++)
    #ifdef SLAVE_GTWI                                          // SLAVE pdomapping setting /////////////////////////////////////
            for(int i=1;i<=NUMOF_GTWI_SLAVES;i++)
            {
                //PO: Pre-Operation, SO: Safe-Operation
                //Link slave's specific setups to PO -> SO
                ec_slave[i].PO2SOconfig = ecat_PDO_Config_GTWI; //Doesn't this function need argument?
                                                           //=> PO2SOconfig is also function.
            }
    #endif
    #ifdef SLAVE_PTWI
            for(int i=NUMOF_GTWI_SLAVES+1;i<=NUMOF_GTWI_SLAVES+NUMOF_PTWI_SLAVES;i++)
            {
                //PO: Pre-Operation, SO: Safe-Operation
                //Link slave's specific setups to PO -> SO
                ec_slave[i].PO2SOconfig = ecat_PDO_Config_PTWI; //Doesn't this function need argument?
                                                           //=> PO2SOconfig is also function.
            }
    #endif

    }
    else
        sigRTthreadKill = 1;
    #endif


// 3. PDO mapping

    ec_config_overlap_map(&IOmap); //Map all PDOs from slaves to IOmap with Outputs/Inputs in sequential order.
    //    ec_config_map(&IOmap);

// 4. Setting Distributed Clock

    int8 dc = ec_configdc(); //Returns ecx_configdc(ecx_contextt *context)=>returns boolean if slaves are found with DC

// 5. Change all slaves pre-OP to SafeOP.

    //ec_statecheck(slave number(0:all slaves), Requested state, Timeout value in microsec)
    ec_statecheck(0, EC_STATE_SAFE_OP, 4*EC_TIMEOUTSTATE); //EC_TIMEOUTSTATE=2,000,000 us

// 6. Calculate expected WKC

    expectedWKC = (ec_group[0].outputsWKC *2) + ec_group[0].inputsWKC; //Calculate WKCs

// 7. Change MASTER state to operational

    ec_slave[0].state = EC_STATE_OPERATIONAL;

// 8. Send one valid process data(Execute PDO communication once)

    ec_send_overlap_processdata(); //Send processdata to slave.
    //    ec_send_processdata();
    wkc = ec_receive_processdata(EC_TIMEOUTRET); //Receive processdata from slaves.


// 9. Request OP state for all slaves

    ec_writestate(0); // Write slave state.

    // 10. PDO data Receive/Transmit

    uint32 obytes = ec_slave[1].Obytes; // Obytes: Output bytes
    uint32 ibytes = ec_slave[1].Ibytes; // Ibytes: Input bytes

    for (int i = 0; i < NUMOFSLAVES; i++) // 0 does not mean master in here
    {
        if (i < NUMOF_GTWI_SLAVES) {
#ifdef SLAVE_GTWI
                // The reason why ec_slave[]has i+1 not i, because ec_slave[0] represents master
                out_twitter_GTWI[i] = (output_GTWI_t *)ec_slave[i + 1].outputs;
                in_twitter_GTWI[i] = (input_GTWI_t *)ec_slave[i + 1].inputs;
#endif
        } else {
#ifdef SLAVE_PTWI
                out_twitter_PTWI[i] = (output_PTWI_t *)ec_slave[i + 1].outputs;
                in_twitter_PTWI[i] = (input_PTWI_t *)ec_slave[i + 1].inputs;
#endif
        }
    }
#ifdef SLAVE_TS
    in_twitter_ts[0] = (input_TS_t *)ec_slave[1].inputs;
#endif

    // 11. Real time

    clock_gettime(CLOCK_MONOTONIC, &expected); //get the system's current time

    tfd = timerfd_create(CLOCK_MONOTONIC, 0); //create timer descriptor
    if(tfd == -1) error(1, errno, "timerfd_create()");

    timer_conf.it_value = expected; // from now
    timer_conf.it_interval.tv_sec = 0;
    timer_conf.it_interval.tv_nsec = RT_PERIOD_MS*1000000; //interval with RT_PERIOD_MS

    int err = timerfd_settime(tfd, TFD_TIMER_ABSTIME, &timer_conf, NULL); //set the timer descriptor
    if(err) error(1, errno, "timerfd_setting()");

    usleep(1000);

///////////////////////////////////////* REALTIME LOOP */////////////////////////////////////

    while(!sigRTthreadKill)
    {
    // 11-1
        clock_gettime(CLOCK_REALTIME, &trt); //get the system time
        t_before_sample_start = trt.tv_nsec;

        err = read(tfd, &ticks, sizeof(ticks)); //read timer and return when the defined interval is reached (periodic)
        clock_gettime(CLOCK_REALTIME, &trt); //get the system time
        if(err<0) error(1, errno, "read()"); //

        old_t1 = t1;
        t1 = trt.tv_nsec;
        if(old_t1>trt.tv_nsec)
        {
            delta_t1 = (t1+1000000000) - old_t1;
        }
        else
        {
            delta_t1 = t1 - old_t1;
        }

        sampling_ms = (double)delta_t1*0.000001; //calculating time interval

        double jitter = sampling_ms - RT_PERIOD_MS; //calculating jitter

        if(ticks>1) overrun += ticks - 1; //calculating total overrun

    // 11-2. Sending processdata (calculated one tick before) => In order to ensure punctuality

        ec_send_overlap_processdata(); //PDO sending
        // ec_send_processdata();

        wkc = ec_receive_processdata(EC_TIMEOUTRET); //returns WKC

        if(expectedWKC>wkc)
        //In case of checked wkc less than WKC that have to be right
        //This means the etherCAT frame cannot be successfully read or
        //wrote on at least one slaves.
        //Up to user
        {
            sigRTthreadKill = 1; //Kill the entire of the RT thread
        }

//////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////* Control loop start *///////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////

        t++;
        
        /**************** Controller DATA set : update ***************/
        for(int i = 0 ; i < 4; i++)
        {
          leg_controller[i].setDelayData();
          leg_kinematics[i].set_DelayDATA();
        }
        
        /**************** Data receive from ELMO ***************/
        
        ACT_FLHAA.DATA_Receive(in_twitter_GTWI);
        ACT_FLHIP.DATA_Receive(in_twitter_GTWI);
        ACT_FLKNEE.DATA_Receive(in_twitter_GTWI);

        ACT_FRHAA.DATA_Receive(in_twitter_GTWI);
        ACT_FRHIP.DATA_Receive(in_twitter_GTWI);
        ACT_FRKNEE.DATA_Receive(in_twitter_GTWI);

        ACT_RLHAA.DATA_Receive(in_twitter_GTWI);
        ACT_RLHIP.DATA_Receive(in_twitter_GTWI);
        ACT_RLKNEE.DATA_Receive(in_twitter_GTWI);

        ACT_RRHAA.DATA_Receive(in_twitter_GTWI);
        ACT_RRHIP.DATA_Receive(in_twitter_GTWI);
        ACT_RRKNEE.DATA_Receive(in_twitter_GTWI);

        leg_motor_acc.col(0) << ACT_FLHIP.getMotor_acc(),ACT_FLKNEE.getMotor_acc();
        leg_motor_acc.col(1) << ACT_FRHIP.getMotor_acc(),ACT_FRKNEE.getMotor_acc();
        leg_motor_acc.col(2) << ACT_RLHIP.getMotor_acc(),ACT_RLKNEE.getMotor_acc();
        leg_motor_acc.col(3) << ACT_RRHIP.getMotor_acc(),ACT_RRKNEE.getMotor_acc();
        
        /****************** Kinematics ******************/
        leg_kinematics[0].Cal_RW(ACT_FLHIP.getMotor_pos(), ACT_FLKNEE.getMotor_pos(), ACT_FLHIP.getMotor_vel(), ACT_FLKNEE.getMotor_vel(), 0);
        leg_kinematics[1].Cal_RW(ACT_FRHIP.getMotor_pos(), ACT_FRKNEE.getMotor_pos(), ACT_FRHIP.getMotor_vel(), ACT_FRKNEE.getMotor_vel(), 1);
        leg_kinematics[2].Cal_RW(ACT_RLHIP.getMotor_pos(), ACT_RLKNEE.getMotor_pos(), ACT_RLHIP.getMotor_vel(), ACT_RLKNEE.getMotor_vel(), 2);
        leg_kinematics[3].Cal_RW(ACT_RRHIP.getMotor_pos(), ACT_RRKNEE.getMotor_pos(), ACT_RRHIP.getMotor_vel(), ACT_RRKNEE.getMotor_vel(), 3);

        leg_kinematics[0].model_param_cal(ACT_FLHIP.getMotor_pos(), ACT_FLKNEE.getMotor_pos());
        leg_kinematics[1].model_param_cal(ACT_FRHIP.getMotor_pos(), ACT_FRKNEE.getMotor_pos());
        leg_kinematics[2].model_param_cal(ACT_RLHIP.getMotor_pos(), ACT_RLKNEE.getMotor_pos());
        leg_kinematics[3].model_param_cal(ACT_RRHIP.getMotor_pos(), ACT_RRKNEE.getMotor_pos());

        // biarticular joint space inertia
        DOB_Lambda_FL = leg_kinematics[0].get_Lambda_nominal_DOB(); DOB_Lambda_FR = leg_kinematics[1].get_Lambda_nominal_DOB();
        DOB_Lambda_RL = leg_kinematics[2].get_Lambda_nominal_DOB(); DOB_Lambda_RR = leg_kinematics[3].get_Lambda_nominal_DOB();

        // RW task space inertia
        FOB_Lambda_FL = leg_kinematics[0].get_Lambda_nominal_FOB(); FOB_Lambda_FR = leg_kinematics[1].get_Lambda_nominal_FOB();
        FOB_Lambda_RL = leg_kinematics[2].get_Lambda_nominal_FOB(); FOB_Lambda_RR = leg_kinematics[3].get_Lambda_nominal_FOB();

        J_FL = leg_kinematics[0].get_RW_Jacobian(); J_FR = leg_kinematics[1].get_RW_Jacobian();
        J_RL = leg_kinematics[2].get_RW_Jacobian(); J_RR = leg_kinematics[3].get_RW_Jacobian();
        JT_FL = leg_kinematics[0].get_RW_Jacobian_Trans(); JT_FR = leg_kinematics[1].get_RW_Jacobian_Trans();
        JT_RL = leg_kinematics[2].get_RW_Jacobian_Trans(); JT_RR = leg_kinematics[3].get_RW_Jacobian_Trans();
        JT_inv_FL = leg_kinematics[0].get_RW_Jacobian_Trans(); JT_inv_FR = leg_kinematics[1].get_RW_Jacobian_Trans();
        JT_inv_RL = leg_kinematics[2].get_RW_Jacobian_Trans(); JT_inv_RR = leg_kinematics[3].get_RW_Jacobian_Trans();
        
        
       for(int i = 0 ; i < 4; i++)
        {
            leg_kinematics[i].pos_trajectory(traj_t,i); 
            leg_RWpos.col(i) = leg_kinematics[i].get_posRW();
            leg_RWvel.col(i) = leg_kinematics[i].get_velRW();
        }
        if(Traj_on) // button stop -> stop time temporally, 
            traj_t += 1 ; 
        
        /****************** State error ******************/ // index 0: curr error/ index 1 : old error
        if(trajectory_test) // trajectory test - simple test
        { 
          for(int i = 0 ; i < 4; i++)
          {
            // for safety
            RW_pos_des = leg_RWpos; RW_vel_des = leg_RWpos;
            // position trajectory
            RW_pos_traj.col(i) = leg_traj[i].test_rw_pos_cos_traj(RW_pos_des,leg_RWpos,traj_interval[i]); // 2d vector
            // velocity trajectory
            RW_vel_traj.col(i) = leg_traj[i].test_rw_vel_cos_traj(RW_vel_des,leg_RWvel,traj_interval[i]);
            // old error update 
            leg_RWpos_err_old.col(i) = leg_RWpos_err.col(i);
            leg_RWvel_err_old.col(i) =  leg_RWvel_err.col(i);
            // error update 
            leg_RWpos_err.col(i) =  RW_pos_traj.col(i) - leg_RWpos.col(i);
            leg_RWvel_err.col(i) =  RW_vel_traj.col(i) - leg_RWvel.col(i);
          }
        }
        else // entire motion control
        {
          for(int i = 0 ; i < 4; i++)
          {            
            leg_RWpos_err.col(i) =  leg_kinematics[i].get_posRW_error(0); // current error
            leg_RWvel_err.col(i) =  leg_kinematics[i].get_velRW_error(0);
            leg_RWpos_err_old.col(i) =  leg_kinematics[i].get_posRW_error(1); // current error
            leg_RWvel_err_old.col(i) =  leg_kinematics[i].get_velRW_error(1);
          }
        }       
       
        /****************** Conrtoller ******************/ // index [0] : R direction output, index [1] : th direction output
        if(Ctrl_on == true){
            if(DOB_on)
            {   // lhs의 값을 이전 값을 사용함.
                leg_d_hat.col(0) = leg_controller[0].DOBRW(leg_RWpid_output.col(0),DOB_Lambda_FL, leg_motor_acc.col(0),150,1);
                leg_d_hat.col(1) = leg_controller[1].DOBRW(leg_RWpid_output.col(1),DOB_Lambda_FR, leg_motor_acc.col(1),150,1);
                leg_d_hat.col(2) = leg_controller[2].DOBRW(leg_RWpid_output.col(2),DOB_Lambda_RL, leg_motor_acc.col(2),150,1);
                leg_d_hat.col(3) = leg_controller[3].DOBRW(leg_RWpid_output.col(3),DOB_Lambda_RR, leg_motor_acc.col(3),150,1);

                leg_controller[0].FOBRW(leg_RWpid_output.col(0),FOB_Lambda_FL, leg_motor_acc.col(0),150,1);
                leg_controller[1].FOBRW(leg_RWpid_output.col(1),FOB_Lambda_FR, leg_motor_acc.col(1),150,1);
                leg_controller[2].FOBRW(leg_RWpid_output.col(2),FOB_Lambda_RL, leg_motor_acc.col(2),150,1);
                leg_controller[3].FOBRW(leg_RWpid_output.col(3),FOB_Lambda_RR, leg_motor_acc.col(3),150,1);


            }
            for(int i = 0 ; i< 4 ; i++ )
            {
                //ctrl_mode = 0 -> pos control //ctrl_mode = 1 -> vel control
                leg_RWpid_output.col(i)(0) = leg_controller[i].pid(leg_RWpos_err.col(i), leg_RWpos_err_old.col(i), leg_RWvel_err.col(i), leg_RWvel_err_old.col(i),0,i,ctrl_mode) + leg_d_hat.col(i)(0); //  index : 0 = r pid 
                leg_RWpid_output.col(i)(1) = leg_controller[i].pid(leg_RWpos_err.col(i), leg_RWpos_err_old.col(i), leg_RWvel_err.col(i), leg_RWvel_err_old.col(i),1,i,ctrl_mode) + leg_d_hat.col(i)(1); //  index : 0 = theta pid 
                
                admit_pos[i] = leg_controller[i].admittance(wn,zeta,k);
            }   
        }
        else // control model가 아닌 경우
        {
          for(int i = 0; i < 4; i++) // not control -> input 0 
            { 
              leg_RWpid_output.col(i) << 0,0;
            }
        }
        
        Homming_input = traj.homming();        
        /****************** Put the torque in Motor ******************/
        for(int i = 0 ; i< 4 ; i++ )
        {
          leg_tau_bi_input.col(i) = JT_FL * leg_RWpid_output.col(i);
          /****************** exchange_mutex ******************/
          leg_controller[i].exchange_mutex();
          leg_kinematics[i].exchange_mutex(i);
        }
        
        // motor exchange        
        ACT_FLHAA.exchange_mutex();
        ACT_FRHAA.exchange_mutex();
        ACT_RLHAA.exchange_mutex();
        ACT_RRHAA.exchange_mutex();
        
        ACT_FLHIP.exchange_mutex();
        ACT_FRHIP.exchange_mutex();
        ACT_RLHIP.exchange_mutex();
        ACT_RRHIP.exchange_mutex();
        
        ACT_FLKNEE.exchange_mutex();
        ACT_FRKNEE.exchange_mutex();
        ACT_RLKNEE.exchange_mutex();
        ACT_RRKNEE.exchange_mutex();
        
        
                /****************** actuator Data send to ELMO ******************/
        ACT_FLHAA.DATA_Send(out_twitter_GTWI);
        ACT_FLHIP.DATA_Send(out_twitter_GTWI);
        ACT_FLKNEE.DATA_Send(out_twitter_GTWI);

        ACT_FRHAA.DATA_Send(out_twitter_GTWI);
        ACT_FRHIP.DATA_Send(out_twitter_GTWI);
        ACT_FRKNEE.DATA_Send(out_twitter_GTWI);

        ACT_RLHAA.DATA_Send(out_twitter_GTWI);
        ACT_RLHIP.DATA_Send(out_twitter_GTWI);
        ACT_RLKNEE.DATA_Send(out_twitter_GTWI);
        
        ACT_RRHAA.DATA_Send(out_twitter_GTWI);
        ACT_RRHIP.DATA_Send(out_twitter_GTWI);
        ACT_RRKNEE.DATA_Send(out_twitter_GTWI);

        // 11-6. Sync data with GUI thread

        
        if (stop == true)
        {
          for(int i = 0; i < 4; i++){
            leg_tau_bi_input.col(i) << 0,0;
          }
        }        
        
        
        //// Data Logging ////
        Logging->data_log();
        
        if(!pthread_mutex_trylock(&data_mut))
        {   
            
            traj.exchange_mutex();
            Logging->exchange_mutex();
            _M_sampling_time_ms = sampling_ms; //when the thread get the mutex, write data into shared global variables
            _M_overrun_cnt = overrun;

            _M_Ecat_WKC = wkc;
            _M_Ecat_expectedWKC = expectedWKC;
            
            /****************** Motor Torque ******************/
            if(!Homming) // 호밍 아닐때 
            {
              // FL : 1
              _M_motor_torque[0] = 0; //constant*0.5;
              _M_motor_torque[1] = leg_tau_bi_input(0,0)*constant;
              _M_motor_torque[2] = leg_tau_bi_input(1,0)*constant;
              
              // FR : 2
              _M_motor_torque[5] = 0; //constant*0.5;
              _M_motor_torque[4] = -leg_tau_bi_input(0,1)*constant;
              _M_motor_torque[3] = -leg_tau_bi_input(1,1)*constant;
             
              // RR : 4
              _M_motor_torque[6] = 0; // - constant*0.5;
              _M_motor_torque[7] = leg_tau_bi_input(0,3)*constant;
              _M_motor_torque[8] = -leg_tau_bi_input(1,3)*constant;
              
              // RL : 3
              _M_motor_torque[9] = 0; //constant*0.5;
              _M_motor_torque[10] = leg_tau_bi_input(0,2)*constant;
              _M_motor_torque[11] = -leg_tau_bi_input(1,2)*constant;
            }
              
            else // 호밍일때 
            {               
                for(int i = 0; i < NUMOFSLAVES; i++)
                { 
                  if(i == 0 || i == 5|| i == 9)
                  _M_motor_torque[i] = Homming_input[i]*constant * 2;
                  else if(i == 6 )
                  _M_motor_torque[i] = -Homming_input[i]*constant * 2;
                  else
                  if(i == 3||i == 4|| i == 8 || i == 11) 
                  {
                    _M_motor_torque[i] = -Homming_input[i]*constant ;
                  }
                  else
                    _M_motor_torque[i] = Homming_input[i]*constant ; 
                }
            }
              
            /*********************** Flag **********************/  
            Homming = _M_Homming_checked;
            Traj_on = _M_Traj_ON; // temp_stop pressed 
            stop = _M_stop;   
            
                    
            /***************** Mode selection ******************/
            ctrl_mode = _M_ctrl_mode;
            
            /***************** DOB clicked ******************/
            DOB_on = _M_DOB_on;
            admittance_on = _M_admittance_on;

            pthread_mutex_unlock(&data_mut);
        }
        t2 = trt.tv_nsec;

    }
    pthread_exit(NULL);
    return NULL;

    //////////////////////////////////////////////////////////////////////////////////////////////////

}


