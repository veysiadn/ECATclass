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
#include "xboxController.h"    



/****************************************************************************/
static ec_master_t           *master = NULL ;
static ec_master_state_t      masterState = {};
static ec_domain_t           *masterDomain = NULL; 
static ec_domain_state_t      masterDomainState = {};  
static char                   slaves_up = 0 ;
static struct timespec        syncTimer ;
/****************************************************************************/
#define PERIODNS  1e6
#define NUM_OF_SLAVES   2

#define TEST_BIT(NUM,N)     (NUM &  (1 << N))
#define SET_BIT(NUM,N)      (NUM |  (1 << N))
#define RESET_BIT(NUM,N)    (NUM & ~(1 << N))

#define TIMESPEC2NS(T)      ((uint64_t) (T).tv_sec * 1e9 + (T).tv_nsec)
#define CLOCK_TO_USE        CLOCK_MONOTONIC

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

static  struct timespec timespec_add(struct timespec time1, struct timespec time2)
{
    struct timespec result;

    if ((time1.tv_nsec + time2.tv_nsec) >= 1e9) {
        result.tv_sec = time1.tv_sec + time2.tv_sec + 1;
        result.tv_nsec = time1.tv_nsec + time2.tv_nsec - 1e9;
    } else {
        result.tv_sec = time1.tv_sec + time2.tv_sec;
        result.tv_nsec = time1.tv_nsec + time2.tv_nsec;
    }

    return result;
}


//VeysiAdn motor states
enum motorStates{
	READY_TO_SWITCH_ON = 1,
	SWITCHED_ON,
	OPERATION_ENABLED,
	FAULT,
	VOLTAGE_ENABLED,
	QUICK_STOP,
	SWITCHED_ON_DISABLED,
	WARNING,
	REMOTE,
	TARGET_REACHED,
	INTERNAL_LIMIT_ACTIVATE
};

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
