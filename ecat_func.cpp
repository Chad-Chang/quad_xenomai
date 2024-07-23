#include "ecat_func.h"
/*  */
boolean needlf;
boolean inOP;
uint8 currentgroup = 0;

/* Functions */
//Used for EtherCAT initialization
//If there was any error, it returns 0.
int ecat_init(const char *ifname) //ifname: Interface name (name of ethernet port)
{
    int i, oloop, iloop, k, wkc_count;
    needlf = FALSE;

    printf("Initializing EtherCAT Master...\n");

    if(ec_init(ifname)>0) // when ec_init is succeeded
    {
        printf("ec_init on %s succeeded.\n", ifname);

        if(ec_config_init(FALSE)>0) //Returns workcounter of slave discover datagram
        {
            printf("%d slaves found and configured.\n", ec_slavecount); //ec_slavecount: number of slaves found on the network

            if(ec_slavecount < NUMOF_SLAVES+1) //Detected slaves are less than actual number of slaves
            {
                return 0;
            }
            if(strcmp(ec_slave[TWITTER_1].name, TWITTER_1_NAME)) //Compare name of slave. ec_slave[0] reserves for master.
            {
                char *nametmp; //Pointer for string. eg) char *s1 = "Hello"; printf("%s\n", s1);=Hello
                nametmp = ec_slave[TWITTER_1].name;
                printf("Name of slave is %s, not TWITTER.\n", nametmp);
                return 0;
            }
        }
        else
        {
            return 0;
        }
    }
    else
    {
        return 0;
    }
    return 1;
}
#ifdef SLAVE_GTWI //Since SLAVE_GTWI is defined at ecat_func.h, codes in here are activated
int ecat_PDO_Config_GTWI(uint16 slave)
{
    int retval_SM2 = 0;
    int retval_SM3 = 0;



        //ec_SDOwrite(slave number, Index to write, Subindex to write(must be 0 or 1, if CA used), CA(T:Complete access all subindexes written//F:single subindex), size in bytes of parameter buffer, pointer to parameter buffer, Timeout in microsec(standard is EC_TIMEOUTRXM))
        //SDOwrite is kind of write in Supervisor mode
        retval_SM2 += ec_SDOwrite(slave, SM2_RXPDO_ASSIGN, 0x00, TRUE, sizeof(RXPDO_ADDR_GTWI), &RXPDO_ADDR_GTWI, EC_TIMEOUTSAFE); //EC_TIMEOUTSAFE: 20000 us
        retval_SM3 += ec_SDOwrite(slave, SM3_TXPDO_ASSIGN, 0x00, TRUE, sizeof(TXPDO_ADDR_GTWI), &TXPDO_ADDR_GTWI, EC_TIMEOUTSAFE);


    return 1;
}
#endif

#ifdef SLAVE_PTWI
int ecat_PDO_Config_PTWI(uint16 slave)
{
    int retval_SM2 = 0;
    int retval_SM3 = 0;

    #ifdef SLAVE_PTWI
        int8 b = 0;
        uint16 w, ind;
        uint32 dw;

        //Motor setting
        uint16 max_torque = 1000;
        uint32 Rated_torque = 12120;
        ec_SDOwrite(slave,0x6072,0,FALSE,sizeof(max_torque),&max_torque,EC_TIMEOUTRXM);
        ec_SDOwrite(slave,0x6073,0,FALSE,sizeof(max_torque),&max_torque,EC_TIMEOUTRXM);
        //rpdo----------------
        //0x1600
        //
        ind = 0;
        ec_SDOwrite(slave, 0x1600, 0, FALSE, sizeof(b), &b, EC_TIMEOUTRXM);
        dw = 0x60400010;
        ec_SDOwrite(slave, 0x1600, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
        dw = 0x60600008;
        ec_SDOwrite(slave, 0x1600, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
        dw = 0x60710010;
        ec_SDOwrite(slave, 0x1600, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
        dw = 0x60fe0120;//60feh( D out )
        ec_SDOwrite(slave, 0x1600, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
//        dw = 0x60720010;
//        ec_SDOwrite(slave, 0x1600, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
//        dw = 0x60730010;
//        ec_SDOwrite(slave, 0x1600, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
//        dw = 0x60750020;
//        ec_SDOwrite(slave, 0x1600, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
//        dw = 0x60760020;
//        ec_SDOwrite(slave, 0x1600, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);

        b=ind; //0x00 => Number of subindexes
        ec_SDOwrite(slave, 0x1600, 0, FALSE, sizeof(b), &b, EC_TIMEOUTRXM);

        //SM2
        retval_SM2 += ec_SDOwrite(slave, SM2_RXPDO_ASSIGN, 0x00, TRUE, sizeof(RXPDO_ADDR_PTWI), &RXPDO_ADDR_PTWI, EC_TIMEOUTSAFE); //EC_TIMEOUTSAFE: 20000 us

        //tpdo----------------

        //1a00
        ind = 0;
        b = 0;
        ec_SDOwrite(slave, 0x1a00, 0, FALSE, sizeof(b), &b, EC_TIMEOUTRXM);
        //dw = htoel_(0x60410010);//6041h( Status word )
        dw = 0x60410010;
        ec_SDOwrite(slave, 0x1a00, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
        //dw = htoel_(0x60610008);//6061h( Mode of Operation Disp )
        dw = 0x60610008;
        ec_SDOwrite(slave, 0x1a00, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
        //dw = htoel_(0x60640020);//6064h ( Position feedback )
        dw = 0x60640020;
        ec_SDOwrite(slave, 0x1a00, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
        //dw = htoel_(0x60770010);//6077h ( Torque feedback )
        dw = 0x60770010;
        ec_SDOwrite(slave, 0x1a00, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
        dw = 0x60790020;
        ec_SDOwrite(slave, 0x1a00, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
//        dw = 0x60fd0020;
//        ec_SDOwrite(slave, 0x1a00, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
//        dw = 0x22050110;
//        ec_SDOwrite(slave, 0x1a00, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
        dw = 0x2fe40240;
        ec_SDOwrite(slave, 0x1a00, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
        dw = 0x606c0020;
        ec_SDOwrite(slave, 0x1a00, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
        b = ind;
        ec_SDOwrite(slave, 0x1a00, 0, FALSE, sizeof(b), &b, EC_TIMEOUTRXM);


        //1c13.0
        //SM3
        b = 0;
        ec_SDOwrite(slave, 0x1c13, 0x00, FALSE, sizeof(b), &b, EC_TIMEOUTRXM);
        w = 0x1a00;
        ec_SDOwrite(slave, 0x1c13, 0x01, FALSE, sizeof(w), &w, EC_TIMEOUTRXM);
        b = 1;
        ec_SDOwrite(slave, 0x1c13, 0x00, FALSE, sizeof(b), &b, EC_TIMEOUTRXM);
          retval_SM3 += ec_SDOwrite(slave, SM3_TXPDO_ASSIGN, 0x00, TRUE, sizeof(TXPDO_ADDR_PTWI), &TXPDO_ADDR_PTWI, EC_TIMEOUTSAFE);

    #endif
    return 1;
}
#endif

#ifdef SLAVE_TS
int ecat_PDO_Config_TS(uint16 slave)
{
    int retval_SM2 = 0;
    int retval_SM3 = 0;

    int8 b = 0;
    uint16 w, ind;
    uint32 dw;
    //tpdo----------------

    //1a01
    ind = 0;
    b = 0;
    ec_SDOwrite(slave, 0x1a01, 0, FALSE, sizeof(b), &b, EC_TIMEOUTRXM);
    dw = 0x20000108;
    ec_SDOwrite(slave, 0x1a01, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
    dw = 0x20000208;
    ec_SDOwrite(slave, 0x1a01, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
    dw = 0x20000308;
    ec_SDOwrite(slave, 0x1a01, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
    dw = 0x20000408;
    ec_SDOwrite(slave, 0x1a01, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);

    dw = 0x20000508;
    ec_SDOwrite(slave, 0x1a01, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
    dw = 0x20000608;
    ec_SDOwrite(slave, 0x1a01, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);

    dw = 0x20000708;
    ec_SDOwrite(slave, 0x1a01, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
    dw = 0x20000808;
    ec_SDOwrite(slave, 0x1a01, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);


    b = ind;
    ec_SDOwrite(slave, 0x1a01, 0, FALSE, sizeof(b), &b, EC_TIMEOUTRXM);

    //1a02
    ind = 0;
    b = 0;
    ec_SDOwrite(slave, 0x1a02, 0, FALSE, sizeof(b), &b, EC_TIMEOUTRXM);
    dw = 0x20000908;
    ec_SDOwrite(slave, 0x1a02, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
    dw = 0x20001008;
    ec_SDOwrite(slave, 0x1a02, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
    dw = 0x20001108;
    ec_SDOwrite(slave, 0x1a02, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
    dw = 0x20001208;
    ec_SDOwrite(slave, 0x1a02, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);

    dw = 0x20001308;
    ec_SDOwrite(slave, 0x1a02, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
    dw = 0x20001408;
    ec_SDOwrite(slave, 0x1a02, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);

    dw = 0x20001508;
    ec_SDOwrite(slave, 0x1a02, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
    dw = 0x20001608;
    ec_SDOwrite(slave, 0x1a02, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);


    b = ind;
    ec_SDOwrite(slave, 0x1a02, 0, FALSE, sizeof(b), &b, EC_TIMEOUTRXM);

    //1a03
    ind = 0;
    b = 0;
    ec_SDOwrite(slave, 0x1a03, 0, FALSE, sizeof(b), &b, EC_TIMEOUTRXM);
    dw = 0x20001708;
    ec_SDOwrite(slave, 0x1a03, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);
    dw = 0x20001808;
    ec_SDOwrite(slave, 0x1a03, ++ind, FALSE, sizeof(dw), &dw, EC_TIMEOUTRXM);


    b = ind;
    ec_SDOwrite(slave, 0x1a03, 0, FALSE, sizeof(b), &b, EC_TIMEOUTRXM);


    //1c13.0
    //SM3
    b = 0;
    ec_SDOwrite(slave, 0x1c13, 0x00, FALSE, sizeof(b), &b, EC_TIMEOUTRXM);
    w = 0x1a00;
    ec_SDOwrite(slave, 0x1c13, 0x01, FALSE, sizeof(w), &w, EC_TIMEOUTRXM);
    b = 1;
    ec_SDOwrite(slave, 0x1c13, 0x00, FALSE, sizeof(b), &b, EC_TIMEOUTRXM);
      retval_SM3 += ec_SDOwrite(slave, SM3_TXPDO_ASSIGN, 0x00, TRUE, sizeof(TXPDO_ADDR_TS), &TXPDO_ADDR_TS, EC_TIMEOUTSAFE);

return 1;
}
#endif
