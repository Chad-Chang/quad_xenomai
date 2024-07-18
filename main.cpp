////////////////////////////////////////////////
///Title: RT_Master_Bimanipulator_Platinum_v1 - main.cpp
///Functions: Multislave+Anybus
///Author: Copyright (C) 2022- Taehoon Kim
///Date: 2022.09.01
///Finished: 2022.09.02
////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////* INDEX *//////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////*

/// 1. INCLUDE
/// 2. MUTEX VARIABLE
/// 3. SETUP FOR RT THREAD
/// 4. DEFINITION FOR SOEM
/// 5. FUNCTION & VARIABLES DECLARATION
/// 6. MAIN FUNCTION
/// 7. FUNCTION DEFINITION
///     1) clean up
///     2) realtime_thread
///         (1) Parameter settingFound
///         (2) EherCAT MASTER
///         (3) Realtime loop
///             - Control loop




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

#include <actuator.h>
#include <controller.h>
#include <data_exchange_mutex.h>
#include <data_mutex.h>
#include <ecat_func.h>
#include <ethercat.h>
#include <filter.h>
#include <kinematics.h>
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

int a = 0 ;
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

Controller C_FL;
Controller C_FR;
Controller C_RL;
Controller C_RR;

// Jacobian
Matrix2d J_FL;
Matrix2d J_FR;
Matrix2d J_RL;
Matrix2d J_RR;


Matrix2d JTrans_FL;
Matrix2d JTrans_FR;
Matrix2d JTrans_RL;
Matrix2d JTrans_RR;

// Controller output : RW PID INPUT//
Vector2d FL_output;
Vector2d FR_output;
Vector2d RL_output;
Vector2d RR_output;

// motor control input
Vector2d FL_PID_output;
Vector2d FR_PID_output;
Vector2d RL_PID_output;
Vector2d RR_PID_output;




//  actuator configuration: HAA, HIP, KNEE
Actuator ACT_RLHAA(5, 0);
Actuator ACT_RLHIP(4, 0.546812);
Actuator ACT_RLKNEE(3, 2.59478);

// modification please
///////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////* actuator  *//////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
//Actuator actuator(motor_num, init_position);

Actuator ACT_RRHAA(0, 0);
Actuator ACT_RRHIP(1, 0.546812);
Actuator ACT_RRKNEE(2, 2.59478);

Actuator ACT_FRHAA(9, 0);
Actuator ACT_FRHIP(10, 0.546812);
Actuator ACT_FRKNEE(11, 2.59478);

Actuator ACT_FLHAA(6, 0);
Actuator ACT_FLHIP(7, 0.546812);
Actuator ACT_FLKNEE(8, 2.59478);
//========================================================== 



Kinematics K_FL;
Kinematics K_FR;
Kinematics K_RL;
Kinematics K_RR;


///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////* Variable declaration *////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////


int t = 0;
/**************trajecotry time for each legs**************/
int traj_t = 0;



///////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////* Safety button*//////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
bool button_start = 0;// 0 before initialized -> temporal stop flag
int ctrl_mode = 0;


// posRW
Vector2d posRW_FL;
Vector2d posRW_FR;
Vector2d posRW_RL;
Vector2d posRW_RR;

Vector2d posRW_err_FL;
Vector2d posRW_err_FR;
Vector2d posRW_err_RL;
Vector2d posRW_err_RR;

Vector2d posRW_err_old_FL;
Vector2d posRW_err_old_FR;
Vector2d posRW_err_old_RL;
Vector2d posRW_err_old_RR;


// velRW
Vector2d velRW_FL;
Vector2d velRW_FR;
Vector2d velRW_RL;
Vector2d velRW_RR;

Vector2d velRW_err_FL;
Vector2d velRW_err_FR;
Vector2d velRW_err_RL;
Vector2d velRW_err_RR;

Vector2d velRW_err_old_FL;
Vector2d velRW_err_old_FR;
Vector2d velRW_err_old_RL;
Vector2d velRW_err_old_RR;

//RWDOB  
Vector2d FL_DOB_output; // 초기값 0으로 setting  해줘야함
Vector2d FR_DOB_output;
Vector2d RL_DOB_output;
Vector2d RR_DOB_output;

void initialize()
{
    FL_DOB_output << 0, 0;
    FR_DOB_output << 0, 0;
    RL_DOB_output << 0, 0;
    RR_DOB_output << 0, 0;

    posRW_err_old_FL << 0, 0;
    posRW_err_old_FR << 0, 0;
    posRW_err_old_RL << 0, 0;
    posRW_err_old_RR << 0, 0;

    velRW_err_old_FL << 0, 0;
    velRW_err_old_FR << 0, 0;
    velRW_err_old_RL << 0, 0;
    velRW_err_old_RR << 0, 0;


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
  MainWindow w;
  Controlwindow c;

  w.show();
  c.show();
  

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

    #ifdef NON_SLAVE_TS
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
    initialize();

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





    // 11-4. Control loop
//////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////* Control loop start *///////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////

        t++;
        
        
        a ++ ; 
        
        
        
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

        /**************** Controller DATA set : update ***************/
        C_FL.setDelayData();
        C_FR.setDelayData();
        C_RL.setDelayData();
        C_RR.setDelayData();
        
        /**************** Kinematic error update ***************/
        K_FL.set_DelayDATA();
        K_FR.set_DelayDATA();
        K_RL.set_DelayDATA();
        K_RR.set_DelayDATA();
        
        


        /****************** Kinematics ******************/
        
        K_FL.Cal_RW(ACT_FLHIP.getMotor_pos(), ACT_FLKNEE.getMotor_pos(), ACT_FLHIP.getMotor_vel(), ACT_FLKNEE.getMotor_vel(), 0);
        K_FR.Cal_RW(ACT_FRHIP.getMotor_pos(), ACT_FRKNEE.getMotor_pos(), ACT_FRHIP.getMotor_vel(), ACT_FRKNEE.getMotor_vel(), 1);
        K_RL.Cal_RW(ACT_RLHIP.getMotor_pos(), ACT_RLKNEE.getMotor_pos(), ACT_RLHIP.getMotor_vel(), ACT_RLKNEE.getMotor_vel(), 2);
        K_RR.Cal_RW(ACT_RRHIP.getMotor_pos(), ACT_RRKNEE.getMotor_pos(), ACT_RRHIP.getMotor_vel(), ACT_RRKNEE.getMotor_vel(), 3);
          
        J_FL = K_FL.get_RW_Jacobian();
        J_FR = K_FR.get_RW_Jacobian();
        J_RL = K_RL.get_RW_Jacobian();
        J_RR= K_RR.get_RW_Jacobian();
        
        // No use JTrans_FL -> using function 
        JTrans_FL = K_FL.get_RW_Jacobian_Trans();
        JTrans_FR = K_FR.get_RW_Jacobian_Trans();
        JTrans_RL = K_RL.get_RW_Jacobian_Trans();
        JTrans_RR = K_RR.get_RW_Jacobian_Trans();
        

        /****************** Trajectory ******************/
        
        K_FL.pos_trajectory(traj_t, 0); // traj_t1 will be updated in the function
        K_FR.pos_trajectory(traj_t, 1); // traj_t2 will be updated in the function
        K_RL.pos_trajectory(traj_t, 2); // traj_t3 will be updated in the function
        K_RR.pos_trajectory(traj_t, 3); // traj_t4 will be updated in the function
        
        if(button_start) // button stop -> stop time temporally, 
          traj_t += 1 ; 
          
        /****************** State ******************/ // pos RW
        
        posRW_FL = K_FL.get_posRW() + C_FL.admittance(30,1,5000); // function의 마지막 input이 0이면 현재 값, 1이면 이전 값
        posRW_FR = K_FR.get_posRW() + C_FR.admittance(30,1,5000);
        posRW_RL = K_RL.get_posRW() + C_RL.admittance(30,1,5000);
        posRW_RR = K_RR.get_posRW() + C_RR.admittance(30,1,5000);
        
//        velRW_FL = K_FL.get_velRW();
        
        
        
        /****************** State error ******************/ // index 0: curr error/ index 1 : old error
        posRW_err_FL = K_FL.get_posRW_error(0);
        posRW_err_FR = K_FR.get_posRW_error(0);
        posRW_err_RL = K_RL.get_posRW_error(0);
        posRW_err_RR = K_RR.get_posRW_error(0);
        
        
        /****************** State error old******************/ // index 0: curr error/ index 1 : old error
        posRW_err_old_FL = K_FL.get_posRW_error(1);
        posRW_err_old_FR = K_FR.get_posRW_error(1);
        posRW_err_old_RL = K_RL.get_posRW_error(1);
        posRW_err_old_RR = K_RR.get_posRW_error(1);
        
//        cout << "posRW_err_FL :" << posRW_err_FL[0] << endl;
//        cout << "posRW_err_RR :" << posRW_err_RR[0] << endl;
//        cout << "posRW_err_old_FL :" << posRW_err_old_FL[0] << endl;
//        cout << "posRW_err_old_RR :" << posRW_err_old_RR[0] << endl;


        /****************** Conrtoller ******************/ // index [0] : R direction output, index [1] : th direction output
        FL_output[0] = C_FL.pid(posRW_err_FL, posRW_err_old_FL, 0, 0,ctrl_mode);
        FL_output[1] = C_FL.pid(posRW_err_FL, posRW_err_old_FL, 1, 0,ctrl_mode);
        
        FR_output[0] = C_FR.pid(posRW_err_FR, posRW_err_old_FR, 0, 1,ctrl_mode);
        FR_output[1] = C_FR.pid(posRW_err_FR, posRW_err_old_FR, 1, 1,ctrl_mode);

        RL_output[0] = C_RL.pid(posRW_err_RL, posRW_err_old_RL, 0, 2,ctrl_mode); // R direction output
        RL_output[1] = C_RL.pid(posRW_err_RL, posRW_err_old_RL, 1, 2,ctrl_mode); // th direction output
                
        RR_output[0] = C_RR.pid(posRW_err_RR, posRW_err_old_RR, 0, 3,ctrl_mode);
        RR_output[1] = C_RR.pid(posRW_err_RR, posRW_err_old_RR, 1, 3,ctrl_mode);
        
        
//******************************* the problem origin****************************
//        cout << "FL_output[0] :" << FL_output[0] << endl;
//        cout << "FL_output[1] :" << FL_output[1] << endl;
//        cout << "RR_output[0] :" << RR_output[0] << endl;
//        cout << "RR_output[1] :" << RR_output[1] << endl;
 

        
        /****************** Put the torque in Motor ******************/
        
        FL_PID_output = JTrans_FL * FL_output;
        FR_PID_output = JTrans_FR * FR_output;
        RL_PID_output = JTrans_RL * RL_output;
        RR_PID_output = JTrans_RR * RR_output;

        // 나중에 이름 고치기. FL_DOT_output 초기값 setting 해줘야함
        FL_DOB_output = FL_PID_output + C_FL.DOBRW(FL_DOB_output, K_FL.get_Lamda_nominal_DOB(),ACT_FLHIP.getMotor_acc(), ACT_FLKNEE.getMotor_acc(), 150, 1);
        FR_DOB_output = FR_PID_output + C_FR.DOBRW(FR_DOB_output, K_FR.get_Lamda_nominal_DOB(), ACT_FRHIP.getMotor_acc(), ACT_FRKNEE.getMotor_acc(), 150, 1);
        RL_DOB_output = RL_PID_output + C_RL.DOBRW(RL_DOB_output, K_RL.get_Lamda_nominal_DOB(), ACT_RLHIP.getMotor_acc(), ACT_RLKNEE.getMotor_acc(), 150, 1);
        RR_DOB_output = RR_PID_output + C_RR.DOBRW(RR_DOB_output, K_RR.get_Lamda_nominal_DOB(), ACT_RRHIP.getMotor_acc(), ACT_RRKNEE.getMotor_acc(), 150, 1);

        C_FL.FOBRW(FL_DOB_output, K_FL.get_Lamda_nominal_FOB(),K_FL.get_RW_Jacobian_Trans() ,ACT_FLHIP.getMotor_acc(), ACT_FLKNEE.getMotor_acc(), 150, 1);
        C_FR.FOBRW(FR_DOB_output, K_FR.get_Lamda_nominal_FOB(),K_FR.get_RW_Jacobian_Trans() ,ACT_FRHIP.getMotor_acc(), ACT_FRKNEE.getMotor_acc(), 150, 1);
        C_RL.FOBRW(RL_DOB_output, K_RL.get_Lamda_nominal_FOB(),K_RL.get_RW_Jacobian_Trans() ,ACT_RLHIP.getMotor_acc(), ACT_RLKNEE.getMotor_acc(), 150, 1);
        C_RR.FOBRW(RR_DOB_output, K_RR.get_Lamda_nominal_FOB(),K_RR.get_RW_Jacobian_Trans() ,ACT_RRHIP.getMotor_acc(), ACT_RRKNEE.getMotor_acc(), 150, 1);

        /****************** Mutex exchange ******************/
        C_FL.Mutex_exchange();
        C_FR.Mutex_exchange();
        C_RL.Mutex_exchange();
        C_RR.Mutex_exchange();
        
        
        K_FL.exchange_mutex(0);
        K_FR.exchange_mutex(1);
        K_RL.exchange_mutex(2);
        K_RR.exchange_mutex(3);
        
        ACT_FLHIP.exchange_mutex();
        ACT_FRHIP.exchange_mutex();
        ACT_RLHIP.exchange_mutex();
        ACT_RRHIP.exchange_mutex();
        
        ACT_FLKNEE.exchange_mutex();
        ACT_FRKNEE.exchange_mutex();
        ACT_RLKNEE.exchange_mutex();
        ACT_RRKNEE.exchange_mutex();
        
        
        
        
//        knee_pos = ACT_RRKNEE.getMotor_pos();
//        if(t == 1 )
//        {
//          offset = knee_pos;
//        }
        
//        knee_pos_error = 0.0001*traj_t- knee_pos;// - (knee_pos - offset);
//        printf("t = %d\n",t);
//        if(a == 1 )
//        {
//          cout <<"nee "<< knee_pos << endl;
//          cout <<"nee_err "<< knee_pos_error << endl;
//        }
        
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

        
        if(!pthread_mutex_trylock(&data_mut))
        {
            _M_sampling_time_ms = sampling_ms; //when the thread get the mutex, write data into shared global variables
            _M_overrun_cnt = overrun;

            _M_Ecat_WKC = wkc;
            _M_Ecat_expectedWKC = expectedWKC;
            
            /****************** Motor Torque ******************/

              _M_motor_torque[0] = 0;
              _M_motor_torque[1] = RR_control_input[0]*1000000.0 / (Torque_constant*Gear_ratio * 45000);
              _M_motor_torque[2] = -RR_control_input[1]*1000000.0 / (Torque_constant*Gear_ratio * 45000);
              
//              _M_motor_torque[2] = (20*knee_pos_error)*1000000.0 / (Torque_constant*Gear_ratio * 45000);
              _M_motor_torque[5] = 0;
              _M_motor_torque[4] = RL_control_input[0]*1000000.0 / (Torque_constant*Gear_ratio * 45000);
              _M_motor_torque[3] = RL_control_input[1]*1000000.0 / (Torque_constant*Gear_ratio * 45000);
             
              _M_motor_torque[6] = 0;
              _M_motor_torque[7] = FL_control_input[0]*1000000.0 / (Torque_constant*Gear_ratio * 45000);
              _M_motor_torque[8] = FL_control_input[1]*1000000.0 / (Torque_constant*Gear_ratio * 45000);
              
              _M_motor_torque[9] = 0;
              _M_motor_torque[10] = -FR_control_input[0]*1000000.0 / (Torque_constant*Gear_ratio * 45000);
              _M_motor_torque[11] = -FR_control_input[1]*1000000.0 / (Torque_constant*Gear_ratio * 45000);
                        
            /****************** Safety button ******************/
            button_start = _M_traj_ON; // temp_stop pressed
            ctrl_mode = _M_ctrl_mode;
 
            pthread_mutex_unlock(&data_mut);
        }
        t2 = trt.tv_nsec;

    }
    pthread_exit(NULL);
    return NULL;

    //////////////////////////////////////////////////////////////////////////////////////////////////

}
