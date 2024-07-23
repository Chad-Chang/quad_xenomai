#ifndef ECAT_FUNC_H
#define ECAT_FUNC_H

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <sys/timerfd.h>
#include <string.h>
#include <malloc.h>
#include <pthread.h>
#include <fcntl.h>
#include <error.h>
#include <errno.h>
#include <sys/mman.h>
#include <rtdm/ipc.h>
#include <inttypes.h>
#include "data_mutex.h"

#include "ethercat.h" //Located in /usr/SOEM/SOEM/soem/ethercat.h

/* ELMO Board type */
//#define SLAVE_PTWI //Gold twitter. In case of Platinum whistle 'SLAVE_PTWI'
#define SLAVE_GTWI
#define NON_SLAVE_TS
//#define SLAVE_TS
/* Number of slaves */
#define NUMOF_SLAVES NUMOF_GTWI_SLAVES
//#define NUMOF_SLAVES NUMOF_PTWI_SLAVES

/* Define number of each slave */
#define TWITTER_1   1

/* Define name of each slave in order to specify slave */
//Able to check this information through 'slaveinfo test program'
#define TWITTER_1_NAME  "ModuleSlotsDrive" //Serial numbers may differ

//Sync Manage
#define SM2_RXPDO_ASSIGN    0x1C12 //Memory location where mapped PDOs are saved
#define SM3_TXPDO_ASSIGN    0x1C13


///* Definition of PDO mapping (Rx, Tx) */
#ifdef SLAVE_GTWI
/// Gold
//Define RxPDO mapping
//See the predefined PDO
//Locates by order that is written
typedef struct PACKED
{
    //int8 = int8_t
    //Entry address: 0x1600
    int32 RXPDO_TARGET_POSITION_DATA;   //subindex: 0x01 //0x607A:0x00(20)
    uint32 RXPDO_DIGITAL_OUTPUTS_DATA;  //subindex: 0x02
    uint16 RXPDO_CONTROLWORD_DATA;  //subindex: 0x03
    //Entry address: 0x1605
    int32 RXPDO_TARGET_POSITION_DATA_0; //_0: same name with the first one //subindex: 0x01
    int32 RXPDO_TARGET_VELOCITY_DATA;   //subindex: 0x02
    int16 RXPDO_TARGET_TORQUE_DATA; //subindex: 0x03
    uint16 RXPDO_MAXIMAL_TORQUE;    //subindex: 0x04
    uint16 RXPDO_CONTROLWORD_DATA_0;    //subindex: 0x05
    int8 RXPDO_MODE_OF_OPERATION_DATA;  //subindex: 0x06
    uint8 RXPDO_RESERVED;   //subindex: 0x07 (Padding)
} output_GTWI_t;

//Define TxPDO mapping
typedef struct PACKED
{
    //Entry address: 0x1A02
    int32 TXPDO_ACTUAL_POSITION_DATA;
    int16 TXPDO_ACTUAL_TORQUE_DATA;
    uint16 TXPDO_STATUSWORD_DATA;
    int8 TXPDO_MODE_OF_OPERATION_DISPLAY_DATA;
    uint8 TXPDO_RESERVED;
    //Entry address: 0x1A03
//    int32 TXPDO_ACTUAL_POSITION_DATA_0;
    uint32 TXPDO_DIGITAL_INPUTS_DATA;
    int32 TXPDO_ACTUAL_VELOCITY_DATA;
//    uint16 TXPDO_STATUSWORD_DATA_0;
//    //Entry address: 0x1A18
//    uint32 TXPDO_DC_LINK_CIRCUIT_VOLTAGE_DATA;
//    //Entry address: 0x1A1D
//    int16 TXPDO_ANALOG_INPUT_1_DATA;
    //Entry address: 0x1A1E
    uint32 TXPDO_AUX_POSITION_ACTUAL_DATA;
} input_GTWI_t;
#endif

#ifdef SLAVE_PTWI
///Platinum PDO
//Define RxPDO mapping
typedef struct PACKED
{
    uint16 RXPDO_CONTROLWORD_DATA;  //0x6040
    int8 RXPDO_MODE_OF_OPERATION_DATA; //0x6060
    int16 RXPDO_TARGET_TORQUE_DATA; //0x6071
    uint32 RXPDO_DIGITAL_OUTPUTS_DATA;  //0x60fe:0x01
//    uint16 RXPDO_MAX_TORQUE;//0X6072
//    uint16 RXPDO_MAX_CURRENT;//0X6073
//    uint32 RXPDO_RATED_CURRENT;//0X6075
//    uint32 RXPDO_RATED_TORQUE;//0X6076



} output_PTWI_t;

//Define TxPDO mapping
typedef struct PACKED
{
    //uint8 a;
    uint16 TXPDO_STATUSWORD_DATA;     //0x6041
    int8 TXPDO_MODE_OF_OPERATION_DISPLAY_DATA; //0x6061
    int32 TXPDO_ACTUAL_POSITION_DATA; //0x6064
    int16 TXPDO_ACTUAL_TORQUE_DATA; //0x6077
    //int16 TXPDO_ACTUAL_CURRENT_DATA; //0x6078
    uint32 TXPDO_DC_LINK_CIRCUIT_VOLTAGE_DATA; //0x6079
//    uint32 TXPDO_DIGITAL_INPUTS_DATA; //0x60fd
//    int16 TXPDO_ANALOG_INPUT_1_DATA; //0x2205:0x01
    double TXPDO_ADDITIONAL_ACTUAL_POSITION_DATA; //0x2FE4:0x02
    int32 TXPDO_ACTUAL_VELOCITY_DATA; //0x606C

} input_PTWI_t;
#endif

#ifdef SLAVE_TS
//Define TxPDO mapping
typedef struct PACKED
{
    uint8 TSIB_0000;
    uint8 TSIB_0001;
    uint8 TSIB_0002;
    uint8 TSIB_0003;

    uint8 TSIB_0004;
    uint8 TSIB_0005;

    uint8 TSIB_0006;
    uint8 TSIB_0007;

    uint8 TSIB_0008;
    uint8 TSIB_0009;

    uint8 TSIB_0010;
    uint8 TSIB_0011;

    uint8 TSIB_0012;
    uint8 TSIB_0013;

    uint8 TSIB_0014;
    uint8 TSIB_0015;

    uint8 TSIB_0016;
    uint8 TSIB_0017;
} input_TS_t;
#endif

/* Data addresses */
//Defined at main.cpp
extern uint16 RXPDO_ADDR_GTWI[3];
extern uint16 TXPDO_ADDR_GTWI[3];

extern uint16 RXPDO_ADDR_PTWI[2];
extern uint16 TXPDO_ADDR_PTWI[2];

extern uint16 TXPDO_ADDR_TS[4];

/* Function pre-definitions */
int ecat_init(const char *ifname);
int ecat_PDO_Config_PTWI(uint16 slave);
int ecat_PDO_Config_GTWI(uint16 slave);
int ecat_PDO_Config_TS(uint16 slave);

#endif // ECAT_FUNC_H
