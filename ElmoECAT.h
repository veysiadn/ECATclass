#pragma once
/*
    03/01/2021 VeysiADN ;  modified example code of IgH EtherCAT Library for
    Elmo Gold Solo Twitter Servo drive, to control Maxon 11830PN, 12V DC brushed motor
    in position mode.
*/

/*
    Notes : Different PDO mapping than default PDO mapping of drive isn't working properly,
    I couldn't succed to move the motor in different mode. e.g. velocity mode or torque mode.
*/
 /*****************************************************************************/

#include <iostream>

#include <limits.h>
#include <stdlib.h>
#include <pthread.h>
#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <time.h>
#include <sys/mman.h>
#include <malloc.h>
#include <sched.h> /* sched_setscheduler() */

/****************************************************************************/
// the user-space real-time interface library.

#include "ecrt.h"                               // IgH EtherCAT library header file, includes all EtherCAT related functions and data types.

#include "objectdictionary.h"                   // Object dictionary paramaters PDO index and default values in here.
/****************************************************************************/
static ec_master_t           *master = NULL ;
static ec_master_state_t     masterState = {};
static ec_domain_t           *masterDomain = NULL; 
static ec_domain_state_t     masterDomainState = {};  

/****************************************************************************/
#define TEST_BIT(NUM,N)     (NUM && (1 << N))
#define TEST_BIT(NUM,N)     (NUM && (1 << N))
#define SET_BIT(NUM,N)      (NUM |  (1 << N))
#define RESET_BIT(NUM,N)    (NUM & ~(1 << N))

//VeysiAdn operationModes
typedef enum
{
    MODE_PROFILE_POSITION = 1,
    MODE_PROFILE_VELOCITY = 3,
    MODE_PROFILE_TORQUE = 4,
    MODE_HOMING = 6,
    MODE_INTERPOLATED_POSITION = 7,
    MODE_CSP = 8,
    MODE_CSV = 9,
    MODE_CST = 10,
} opmode_t ;

//VeysiAdn motor states
enum
{
    STATE_INIT  = 0,
    STATE_ENABLING = 1,
    STATE_ENABLED = 2,
    STATE_MOVING  = 3,
    STATE_TARGET_REACHED = 4,
    STATE_FAULT = 5,

    STATE_CSP = 8,
    STATE_CSV = 9,
    STATE_CST = 10,

}motor_state{STATE_INIT};

//VeysiAdn offset for pdo entries
typedef struct
{
    unsigned int target_pos ;
    unsigned int target_vel ;
    unsigned int target_tor ;
    unsigned int max_tor ;
    unsigned int control_word ;
    unsigned int mode ;
    unsigned int vel_offset ;
    unsigned int tor_offset ;

    unsigned int actual_pos ;
    unsigned int actual_vel ;
    unsigned int actual_cur ;
    unsigned int actual_tor ;
    unsigned int status_word ;
    unsigned int mode_display ;
} offset_t ;


// VeysiAdn received data
typedef struct
{
    int32_t  target_pos ;
    int32_t  target_vel ;
    int16_t  target_tor ;
    int16_t  max_tor ;
    uint16_t control_word ;
    opmode_t mode ;
    int32_t  vel_offset ;
    int16_t  tor_offset ;

    int32_t  actual_pos ;
    int32_t  actual_vel ;
    int16_t  actual_cur ;
    int16_t  actual_tor ;
    uint16_t status_word ;
    int8_t   mode_display ;
}data_t;
//CKim - ProfilePositionParameters
typedef struct 
{
    uint32_t profileVelocity ;
    uint32_t profileAcceleration ;
    uint32_t profileDeceleration ;
    uint32_t maxFollowingError ;
    uint32_t maxProfileVelocity ; 
    uint32_t quickStopDeceleration ;
    uint16_t motionProfileType ; 

} ProfilePosParam ;

// CKim - Homing parameters
typedef struct
{
	uint32_t	MaxFollowingError;		// 6065 0
	uint32_t	MaxProfileVelocity;		// 607F 0
	uint32_t	QuickStopDecel;			// 6085 0
	uint32_t	SpeedForSwitchSearch;	// 6099 01
	uint32_t	SpeedForZeroSearch;		// 6099 02
	uint32_t	HomingAccel;			// 609A 0
	uint16_t	CurrentThresholdHoming;	// 2080		// Used when homing by touching mechanical limit and sensing current
	int32_t		HomeOffset;				// 607c		// Amount to move away from the sensed limit
	int8_t		HomingMethod;			// 6098
} HomingParam;
// CKim - ProfileVelocityParameters.
typedef struct
{
    uint32_t	MaxProfileVelocity;		// 607F 0
    uint32_t	QuickStopDecel;			// 6085 0
    uint32_t	ProfileAccel;			// 6083 0
    uint32_t	ProfileDecel;			// 6084 0
    uint16_t    MotionProfileType;      // 6086 0
} ProfileVelocityParam ;

typedef struct
{
    ec_sdo_request *s_ProfileAcceleration ;
    ec_sdo_request *s_ProfileDeceleration ;
    ec_sdo_request *s_ProfileVelocity ;
    ec_sdo_request *s_QuickStopDeceleration ;
    ec_sdo_request *s_MotionProfileType ;
    ec_sdo_request *s_MaxProfileVelocity ;
    ec_sdo_request *s_maxFollowingError ;
} sdoRequest_t ;


class ElmoECAT{
public:
/**********************************************************************************/
    static const uint16_t alias_ = 0 ; 
    uint16_t position_ ; 
    static const uint32_t vendorId_    = 0x0000009a ; 
    static const uint32_t productCode_ = 0x00030924 ;

    ec_slave_config_t  *slaveConfig ;
    offset_t            offset;
    data_t              data;
    uint32_t cycleTime;
    uint32_t sync0_shift;

    const static ec_pdo_entry_reg_t profilePosition_PdoRegs[] = {
        {alias_, position_, vendorId_,productCode_,od_positionActualVal, offset.actual_pos},
        {alias_, position_, vendorId_,productCode_,od_velocityActvalue,  offset.actual_vel},
        {alias_, position_, vendorId_,productCode_,od_currentActualValue, offset.actual_cur},
        {alias_, position_, vendorId_,productCode_,od_positionActualVal, offset.actual_pos},
        {alias_, position_, vendorId_,productCode_,od_positionActualVal, offset.actual_pos}
        {alias_, position_, vendorId_,productCode_,od_positionActualVal, offset.actual_pos},
        {}
    };
    /*******************************************************************************/
    //RxPdo
    /*static ec_pdo_entry_info_t motor_rxpdo_entries[8] =
    {
        {0x607a, 0x00, 32}, //pos_target_value  s32
        {0x60ff, 0x00, 32}, //vel_target_value  s32
        {0x6071, 0x00, 16}, //tor_target_value  s16
        {0x6072, 0x00, 16}, //tor_max_value     s16
        {0x6040, 0x00, 16}, //control_word      u16
        {0x6060, 0x00, 8},  //op_mode            u8

        {0x60b1, 0x00, 32}, //velocity offset   s32
        {0x60b2, 0x00, 16}, //torque_offset     s16
    };
   static ec_pdo_info_t       motor_rxpdos[3] =
    {
        {0x1605, 6, motor_rxpdo_entries + 0},
        {0x1617, 1, motor_rxpdo_entries + 6},
        {0x1618, 1, motor_rxpdo_entries + 7},
    };
    // TxPdo
    static ec_pdo_entry_info_t motor_txpdo_entries[6] =
    {
        {0x6064, 0x00, 32}, //pos_actual_value  s32
        {0x606c, 0x00, 32}, //vel_actual_value  s32
        {0x6078, 0x00, 16}, //cur_actual_value  s16
        {0x6077, 0x00, 16}, //tor_actual_value  s16

        {0x6041, 0x00, 16}, //status_word       u16
        {0x6061, 0x00, 8},  //mode_display    u8
    };
    static ec_pdo_info_t motor_txpdos[6] =
    {
        {0x1a0e, 1, motor_txpdo_entries + 0}, //pos_actual_value  s32
        {0x1a11, 1, motor_txpdo_entries + 1}, //vel_actual_value  s32
        {0x1a1f, 1, motor_txpdo_entries + 2}, //cur_actual_value  s16
        {0x1a13, 1, motor_txpdo_entries + 3}, //tor_actual_value  s16

        {0x1a0a, 1, motor_txpdo_entries + 4}, //status_word       u16
        {0x1a0b, 1, motor_txpdo_entries + 5}, //module_display    u8
    };
    static ec_sync_info_t syncs[3] = {
        {2, EC_DIR_OUTPUT, 3, motor_rxpdos, EC_WD_ENABLE},
        {3, EC_DIR_INPUT,  6, motor_txpdos, EC_WD_ENABLE},
        {0xff}
    };*/

    ElmoECAT();
    ~ElmoECAT();

    void ConfigureMaster();
    void ConfigureSlave(uint16_t pos);
    void SetProfilePositionPdoRegs(uint16_t  pos);
    int MapPDOs(ec_sync_info_t *syncs, ec_pdo_entry_reg_t *pdo_entry_reg);
    void ConfigDCSync();
    int ConfigSDORequests(sdoRequest_t& e_sdo);
    int ReadSDO(ec_sdo_request_t *req, uint32_t& target);
    void WriteSDO(ec_sdo_request_t *req, uint32_t data);
    void SetOperationMode(uint8_t om);
    int GetProfilePositionParameters (ProfilePosParam& P, sdoRequest_t& sr);
    int SetProfilePositionParameters( ProfilePosParam& P );
    int SetProfileVelocityParameters(ProfileVelocityParam& P);
    void  ActivateMaster();
    void ResetMaster();
    int SetMode(uint8_t *slavePdoDomain, int8_t mode_);


/*
    int GetProfilePositionParameters(ProfilePosParam& P );
    int MoveTargetPosition(uint8_t *slavePdoDomain);


    int Display(uint8_t *slavePdoDomain);
    
    int SetTargtVelocity(uint8_t *slavePdoDomain, int32_t velocity_);
    
    int EnableDevice(uint8_t *slavePdoDomain);
    int DisableDevice(uint8_t *slavePdoDomain);

    uint8_t GetMode();
    int32_t GetActualPosition();
    int32_t GetActualVelocity();
    int16_t GetActualCurrent();
    

    int SetTargtPosition(int32_t position);
    int SetTargtTorque(int16_t torque);
*/

};

