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
// IgH EtherCAT library header file, includes all EtherCAT related functions and data types.
#include "ecrt.h"                     
// Object dictionary paramaters PDO index and default values in here.
#include "objectdictionary.h"         
/****************************************************************************/
static ec_master_t           *master = NULL ;
static ec_master_state_t      masterState = {};
static ec_domain_t           *masterDomain = NULL; 
static ec_domain_state_t      masterDomainState = {};  
static char                   slaves_up = 0 ;
static struct timespec               syncTimer ;
/****************************************************************************/
#define TEST_BIT(NUM,N)     (NUM && (1 << N))
#define TEST_BIT(NUM,N)     (NUM && (1 << N))
#define SET_BIT(NUM,N)      (NUM |  (1 << N))
#define RESET_BIT(NUM,N)    (NUM & ~(1 << N))
#define NUM_OF_SLAVES   1
#define TIMESPEC2NS(T) ((uint64_t) (T).tv_sec * 1e9 + (T).tv_nsec)
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
    unsigned int profileAcc ;
    unsigned int profileDec ;
    unsigned int quicStopDec ;
    unsigned int profileVel ;

    unsigned int actual_pos ;
    unsigned int posFollowingError ;
    unsigned int actual_vel ;
    unsigned int actual_cur ;
    unsigned int actual_tor ;
    unsigned int status_word ;
    unsigned int mode_display ;
    unsigned int dcCircuitLinkVoltage ;
    unsigned int errorCode ;
    unsigned int extraStatusReg ;
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

    ec_slave_config_t       *slaveConfig ;
    ec_slave_config_state_t slaveConfigState ;
    offset_t            offset ;
    data_t              data ;
    uint8_t             *slavePdoDomain ;
    uint32_t cycleTime ;
    uint32_t sync0_shift ;
    pthread_t motorThread;
    static pthread_t XboxControllerThread;
    ec_pdo_entry_reg_t masterDomain_PdoRegs[21] = {
        // Input PDO mapping ; 
        {alias_, position_, vendorId_,productCode_, od_positionActualVal ,        &offset.actual_pos},
        {alias_, position_, vendorId_,productCode_, od_positonFollowingError ,    &offset.posFollowingError},
        {alias_, position_, vendorId_,productCode_, od_torqueActualValue ,        &offset.actual_tor},
        {alias_, position_, vendorId_,productCode_, od_statusWord ,               &offset.status_word},
        {alias_, position_, vendorId_,productCode_, od_operationModeDisplay ,     &offset.mode_display},
        {alias_, position_, vendorId_,productCode_, od_velocityActvalue ,         &offset.actual_vel},
        {alias_, position_, vendorId_,productCode_, od_currentActualValue ,       &offset.actual_cur},
        {alias_, position_, vendorId_,productCode_, od_dcCircuitLinkVoltage ,     &offset.dcCircuitLinkVoltage},
        {alias_, position_, vendorId_,productCode_, od_errorCode ,                &offset.errorCode},
        {alias_, position_, vendorId_,productCode_, od_extraStatusRegister ,      &offset.extraStatusReg},

        // Output PDO Mapping ; 
        {alias_, position_, vendorId_,productCode_,od_targetPosition,            &offset.target_pos},
        {alias_, position_, vendorId_,productCode_,od_targetVelocity ,           &offset.target_vel},
        {alias_, position_, vendorId_,productCode_,od_targetTorque,              &offset.target_tor},
        {alias_, position_, vendorId_,productCode_,od_torqueMax ,                &offset.max_tor},
        {alias_, position_, vendorId_,productCode_,od_controlWord ,              &offset.control_word},
        {alias_, position_, vendorId_,productCode_,od_operationMode ,            &offset.mode},
        {alias_, position_, vendorId_,productCode_,od_profileVelocity ,          &offset.profileVel},
        {alias_, position_, vendorId_,productCode_,od_profileAcceleration ,      &offset.profileAcc},
        {alias_, position_, vendorId_,productCode_,od_profileDeceleration ,      &offset.profileDec},
        {alias_, position_, vendorId_,productCode_,od_quickStopDeceleration ,    &offset.quicStopDec},
        {}
    };
    /*******************************************************************************/
    ec_pdo_entry_info_t GS_PDO_Entries[22] = {
     // Outputs from master to slave , WRITE to this indexes
    {od_targetPosition, 32},    // TARGET_POSITION
    {od_targetVelocity, 32},    // TARGET_VELOCITY
    {od_targetTorque,   16},    // TARGET_TORQUE
    {od_torqueMax,      16},    // MAX_TORQUE
    {od_controlWord,    16},    // CONTROL_WORD
    {od_operationMode,   8},     // MODE_OF_OPERATION
    {0x0000, 0x00,    8}, /* Gap */
    {od_profileVelocity,        32},     // PROFILE_VELOCITY
    {od_profileAcceleration,    32},     // PROFILE_ACCELERATION
    {od_profileDeceleration,    32},     // PROFILE_DECELERATION
    {od_quickStopDeceleration,  32},     // QUICK_STOP_DECELERATION
    //{0x60b1, 0x00, 32},     // VELOCITY_OFFSET

    // Inputs from slave to the master READ from this indexes
    {od_positionActualVal,     32},     // POSITION_ACTUAL_VALUE
    {od_positonFollowingError, 32},     // POSITION_FOLLOWING_ERROR_ACTUAL_VALUE
    {od_torqueActualValue, 16},        // TORQUE_ACTUAL_VALUE
    {od_statusWord,        16},        // STATUS_WORD
    {od_operationModeDisplay, 8},      // CUR_MODE_OF_OPERATION
    {0x0000, 0x00, 8}, /* Gap */
    {od_velocityActvalue,    32},      // VELOCITY_SENSOR_ACTUAL_VALUE (COUTNS/SEC)
    {od_dcCircuitLinkVoltage, 32},             //DC Link Circuit Voltage
    {od_currentActualValue, 16},        // CURRENT_ACTUAL_VALUE
    {od_extraStatusRegister, 16},             // EXTRA_STATUS_REGISTER
    {od_errorCode,    16},             // ERROR_CODE
};

    ec_pdo_info_t GS_PDO_Indexes[11] = {
        {0x1605, 7, GS_PDO_Entries + 0},   //16XX From master to slave outputs e.g Target Position RxPDO
        {0x1611, 1, GS_PDO_Entries + 7}, 
        {0x1613, 1, GS_PDO_Entries + 8},
        {0x1614, 1, GS_PDO_Entries + 9},
        {0x161f, 1, GS_PDO_Entries + 10},

        {0x1a04, 6, GS_PDO_Entries + 11},   //1AXX From slave to the master inputs e.g Actual Position TxPDO
        {0x1a0f, 1, GS_PDO_Entries + 17},
        {0x1a18, 1, GS_PDO_Entries + 18},
        {0x1a1f, 1, GS_PDO_Entries + 19},
        {0x1a21, 1, GS_PDO_Entries + 20},
        {0x1a26, 1, GS_PDO_Entries + 21},
    };

    ec_sync_info_t GS_Syncs[5] = {
        {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
        {1, EC_DIR_INPUT,  0, NULL, EC_WD_DISABLE},
        {2, EC_DIR_OUTPUT, 5, GS_PDO_Indexes + 0, EC_WD_ENABLE},
        {3, EC_DIR_INPUT,  6, GS_PDO_Indexes + 5, EC_WD_DISABLE},
        {0xff}
    };


    ElmoECAT();
    ~ElmoECAT();

    int ConfigureMaster();
    int  ConfigureSlave(uint16_t pos);
    void SetProfilePositionPdoRegs(uint16_t  pos);
    int  MapPDOs(ec_sync_info_t *syncs, ec_pdo_entry_reg_t *pdo_entry_reg);
    void ConfigDCSync();
    int  ConfigSDORequests(sdoRequest_t& e_sdo);
    int  ReadSDO(ec_sdo_request_t *req, uint32_t& target);
    void WriteSDO(ec_sdo_request_t *req, uint32_t data);
    void SetOperationMode(uint8_t om);
    
    void CheckSlaveConfigurationState();
    int  CheckMasterState();
    void CheckMasterDomainState();
    void WaitForOPmode();
    void StartRealTimeTasks();
    void* ReadXboxValues(void *arg);
    void* MotorCyclicTask(void *arg);

    int  GetProfilePositionParameters (ProfilePosParam& P, sdoRequest_t& sr);
    int  SetProfilePositionParameters( ProfilePosParam& P );
    int  SetProfileVelocityParameters(ProfileVelocityParam& P);
    int ActivateMaster();
    void ResetMaster();


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

