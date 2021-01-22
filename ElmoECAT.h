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
#include "globals.h"

class ElmoECAT{
public:
/**********************************************************************************/
    static const uint16_t alias_ = 0 ; 
    uint16_t              position_ ; 
    static const uint32_t vendorId_    = 0x0000009a ; 
    static const uint32_t productCode_ = 0x00030924 ;
    static int            positionCount ; 
    ec_slave_config_t       *slaveConfig ;
    ec_slave_config_state_t  slaveConfigState ;

    offset_t                 offset ;
    data_t                   data ;

    uint8_t               *slavePdoDomain ;

    uint32_t                 cycleTime ;
    uint32_t                 sync0_shift ;
    int32_t                  c_motorState ;
    pthread_t                motorThread;
    pthread_t                XboxControllerThread;

    static XboxController           Controller;
    
    ProfileVelocityParam    VelocityParam ;
    ProfilePosParam         PositionParam ; 

    ec_pdo_entry_reg_t masterDomain_PdoRegs[7] = {
        // Input PDO mapping ; 
        {alias_, position_, vendorId_,productCode_, od_positionActualVal ,        &this->offset.actual_pos},
        {alias_, position_, vendorId_,productCode_, od_statusWord ,               &this->offset.status_word},
        {alias_, position_, vendorId_,productCode_, od_velocityActvalue ,         &this->offset.actual_vel},
        
     /*   {alias_, position_, vendorId_,productCode_, od_operationModeDisplay ,     &this->offset.mode_display},
        {alias_, position_, vendorId_,productCode_, od_positonFollowingError ,    &this->offset.posFollowingError},
        {alias_, position_, vendorId_,productCode_, od_torqueActualValue ,        &this->offset.actual_tor},
        {alias_, position_, vendorId_,productCode_, od_currentActualValue ,       &this->offset.actual_cur},
        {alias_, position_, vendorId_,productCode_, od_dcCircuitLinkVoltage ,     &this->offset.dcCircuitLinkVoltage},
        {alias_, position_, vendorId_,productCode_, od_errorCode ,                &this->offset.errorCode},
        {alias_, position_, vendorId_,productCode_, od_extraStatusRegister ,      &this->offset.extraStatusReg},*/

        // Output PDO Mapping ; 
        {alias_, position_, vendorId_,productCode_,od_targetPosition,            &this->offset.target_pos},
        {alias_, position_, vendorId_,productCode_,od_targetVelocity ,           &this->offset.target_vel},
        {alias_, position_, vendorId_,productCode_,od_controlWord ,              &this->offset.control_word},
        /*
        {alias_, position_, vendorId_,productCode_,od_targetTorque,              &this->offset.target_tor},
        {alias_, position_, vendorId_,productCode_,od_torqueMax ,                &this->offset.max_tor},
        {alias_, position_, vendorId_,productCode_,od_operationMode ,            &this->offset.mode},
        {alias_, position_, vendorId_,productCode_,od_profileVelocity ,          &this->offset.profileVel},
        {alias_, position_, vendorId_,productCode_,od_profileAcceleration ,      &this->offset.profileAcc},
        {alias_, position_, vendorId_,productCode_,od_profileDeceleration ,      &this->offset.profileDec},
        {alias_, position_, vendorId_,productCode_,od_quickStopDeceleration ,    &this->offset.quicStopDec},*/
        {}
    };
    /*******************************************************************************/
    ec_pdo_entry_info_t GS_PDO_Entries[8] = {
     // Outputs from master to slave , WRITE to this indexes
  /*  {od_targetPosition, 32},    // TARGET_POSITION
    {od_targetVelocity, 32},    // TARGET_VELOCITY
    {od_targetTorque,   16},    // TARGET_TORQUE
    {od_torqueMax,      16},    // MAX_TORQUE
    {od_controlWord,    16},    // CONTROL_WORD
    {od_operationMode,   8},     // MODE_OF_OPERATION
    {0x0000, 0x00,    8},       //Gap 
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
    {0x0000, 0x00, 8},                  // Gap 
    {od_velocityActvalue,    32},      // VELOCITY_SENSOR_ACTUAL_VALUE (COUTNS/SEC)
    {od_dcCircuitLinkVoltage, 32},             //DC Link Circuit Voltage
    {od_currentActualValue, 16},        // CURRENT_ACTUAL_VALUE
    {od_extraStatusRegister, 16},             // EXTRA_STATUS_REGISTER
    {od_errorCode,    16},             // ERROR_CODE
    */


    {od_targetPosition, 32},
    {od_digitalOutputs, 32},
    {od_controlWord, 16},
    {od_targetVelocity,32},
    {od_positionActualVal, 32},
    {od_digitalInputs, 32},
    {od_statusWord, 16},
    {od_velocityActvalue,32}
};

    ec_pdo_info_t GS_PDO_Indexes[4] = {
        //16XX From master to slave outputs e.g Target Position RxPDO
      /*{0x1605, 7, GS_PDO_Entries + 0},   
        {0x1611, 1, GS_PDO_Entries + 7}, 
        {0x1613, 1, GS_PDO_Entries + 8},
        {0x1614, 1, GS_PDO_Entries + 9},
        {0x161f, 1, GS_PDO_Entries + 10},

        //1AXX From slave to the master inputs e.g Actual Position TxPDO
        {0x1a04, 6, GS_PDO_Entries + 11},   
        {0x1a0f, 1, GS_PDO_Entries + 17},
        {0x1a18, 1, GS_PDO_Entries + 18},
        {0x1a1f, 1, GS_PDO_Entries + 19},
        {0x1a21, 1, GS_PDO_Entries + 20},
        {0x1a26, 1, GS_PDO_Entries + 21}, */
        {0x1600, 3, GS_PDO_Entries + 0},
        {0x1607, 1, GS_PDO_Entries + 3},
    
        {0x1a00, 3, GS_PDO_Entries + 4},
        {0x1a07, 1, GS_PDO_Entries + 7}
    };

    ec_sync_info_t GS_Syncs[5] = {
        /*{0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
        {1, EC_DIR_INPUT,  0, NULL, EC_WD_DISABLE},
        {2, EC_DIR_OUTPUT, 5, GS_PDO_Indexes + 0, EC_WD_ENABLE},
        {3, EC_DIR_INPUT,  6, GS_PDO_Indexes + 5, EC_WD_DISABLE},
        {0xff}*/
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 2, GS_PDO_Indexes + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 2, GS_PDO_Indexes + 2, EC_WD_DISABLE},
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
    int  WaitForOPmode();
    void EnableDevice();
    int StartRealTimeTasks(ElmoECAT c);
    void *ReadXboxValues(void *arg);
    static void* HelperReadXboxValues(void *context);
    static void* HelperMotorCyclicTask(void *context);
    void *MotorCyclicTask(void *arg);

    int  PrepareForProfilePositionMode();
    void  GetDefaultPositionParameters();
    int  GetProfilePositionParameters (ProfilePosParam& P, sdoRequest_t& sr);
    int  SetProfilePositionParameters( ProfilePosParam& P );

    int PrepareForProfileVelocityMode();
    void GetDefaultVelocityParameters();
    int  SetProfileVelocityParameters(ProfileVelocityParam& P);
    
    int ActivateMaster();
    int RegisterDomain()
    int KeepInOPmode();
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

