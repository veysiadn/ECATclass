#include "ElmoECAT.h"

ElmoECAT::ElmoECAT()
{

}

ElmoECAT::~ElmoECAT()
{
    ecrt_master_deactivate(master);
    ecrt_release_master(master);
}




int ElmoECAT::ConfigureMaster()
{
    master = ecrt_request_master(0);    
    if (!master) {
        std::cout << "Failed to request master instance ! " << std::endl;
        return -1 ;
    }
    masterDomain = ecrt_master_create_domain(master);
    if(!masterDomain) {
        std::cout << "Failed to create master domain ! " << std::endl;
        return -1 ;
    }
    return 0 ;
}
// before calling this function you have to call ConfigureMaster() function \
    and after that you have to specifiy your slaves alias, position, vendorId, and product code\
    this information can be obtained via writing " $ ethercat cstruct " to command line.
int ElmoECAT::ConfigureSlave(uint16_t pos)
{
    slaveConfig = ecrt_master_slave_config(master,alias_,pos,vendorId_,productCode_);
    if(!slaveConfig) {
        std::cout << "Failed to  configure slave ! " << std::endl;
        return ;
    }
}

void ElmoECAT::SetProfilePositionPdoRegs(uint16_t  pos)
{

}

int ElmoECAT::MapPDOs(ec_sync_info_t *syncs, ec_pdo_entry_reg_t *pdo_entry_reg)
{
    int err = ecrt_slave_config_pdos(slaveConfig,EC_END,syncs);
    if ( err ) {
        std::cout <<  "Failed to configure  PDOs! " << std::endl ;
        return -1;
    } 
    err = ecrt_domain_reg_pdo_entry_list(masterDomain, pdo_entry_reg);
    if ( err ){
        std::cout << "Failed to register PDO entries" << std::endl;
        return -1;
    }
    return 0;
}

void ElmoECAT::ConfigDCSync()
{
    return ecrt_slave_config_dc(slaveConfig, 0x0300, cycleTime, sync0_shift, 0, 0);
}

int ElmoECAT::ConfigSDORequests(sdoRequest_t& e_sdo)
{    
    if (!(e_sdo.s_maxFollowingError = ecrt_slave_config_create_sdo_request
                    (slaveConfig, od_maxFollowingError, 4))) {
        fprintf(stderr, "Failed to create SDO request.\n");
        return -1;
    }
    ecrt_sdo_request_timeout(e_sdo.s_maxFollowingError, 500); // ms

    if (!(e_sdo.s_MaxProfileVelocity = ecrt_slave_config_create_sdo_request
                    (slaveConfig, od_maxProfileVelocity, 4))) {
        fprintf(stderr, "Failed to create SDO request.\n");
        return -1;
    }
    ecrt_sdo_request_timeout(e_sdo.s_MaxProfileVelocity, 500); // ms

    if (!(e_sdo.s_MotionProfileType = ecrt_slave_config_create_sdo_request
                    (slaveConfig, od_motionProfileType, 2))) {
        fprintf(stderr, "Failed to create SDO request.\n");
        return -1;
    }
    ecrt_sdo_request_timeout(e_sdo.s_MotionProfileType, 500); // ms      

    if (!(e_sdo.s_ProfileAcceleration = ecrt_slave_config_create_sdo_request
                    (slaveConfig, od_profileAcceleration, 4))) {
        fprintf(stderr, "Failed to create SDO request.\n");
        return -1;
    }
    ecrt_sdo_request_timeout(e_sdo.s_ProfileAcceleration, 500); // ms

    if (!(e_sdo.s_ProfileDeceleration = ecrt_slave_config_create_sdo_request
                    (slaveConfig, od_profileDeceleration, 4))) {
        fprintf(stderr, "Failed to create SDO request.\n");
        return -1;
    }
    ecrt_sdo_request_timeout(e_sdo.s_ProfileDeceleration, 500); // ms
    if (!(e_sdo.s_maxFollowingError = ecrt_slave_config_create_sdo_request
                    (slaveConfig, od_profileDeceleration, 4))) {
        fprintf(stderr, "Failed to create SDO request.\n");
        return -1;
    }
    ecrt_sdo_request_timeout(e_sdo.s_ProfileVelocity, 500); // ms
    if (!(e_sdo.s_maxFollowingError = ecrt_slave_config_create_sdo_request
                    (slaveConfig, od_profileVelocity, 4))) {
        fprintf(stderr, "Failed to create SDO request.\n");
        return -1;
    }
    ecrt_sdo_request_timeout(e_sdo.s_ProfileVelocity, 500); // ms
    if (!(e_sdo.s_QuickStopDeceleration = ecrt_slave_config_create_sdo_request
                    (slaveConfig, od_quickStopDeceleration, 4))) {
        fprintf(stderr, "Failed to create SDO request.\n");
        return -1;
    }
    ecrt_sdo_request_timeout(e_sdo.s_QuickStopDeceleration, 500); // ms
}

int ElmoECAT::ReadSDO(ec_sdo_request_t *req, uint32_t& target)
{
    switch (ecrt_sdo_request_state(req)) {
    case EC_REQUEST_UNUSED:                                      // request was not used yet
        ecrt_sdo_request_read(req);                             // trigger first read
        break;
    case EC_REQUEST_BUSY:
        fprintf(stderr, "SDO still busy...\n");
        break;
    case EC_REQUEST_SUCCESS:
        target = EC_READ_U32(ecrt_sdo_request_data(req));
        ecrt_sdo_request_read(req);                             // trigger next read
        break;
    case EC_REQUEST_ERROR:
        fprintf(stderr, "Failed to read SDO!\n");   
        ecrt_sdo_request_read(req);                              // retry reading
        break;
    }
    return target;
}

void ElmoECAT::WriteSDO(ec_sdo_request_t *req, uint32_t data)
{
	EC_WRITE_U32(ecrt_sdo_request_data(req), data & 0xffffffff);

	switch (ecrt_sdo_request_state(req)) {
		case EC_REQUEST_UNUSED:                      // request was not used yet
			ecrt_sdo_request_write(req);            // trigger first read
			break;
		case EC_REQUEST_BUSY:
			fprintf(stderr, "SDO write still busy...\n");
			break;
		case EC_REQUEST_SUCCESS:
			ecrt_sdo_request_write(req);
			break;
		case EC_REQUEST_ERROR:
			fprintf(stderr, "Failed to write SDO!\n");
			ecrt_sdo_request_write(req);            // retry writing
			break;
	}
}

void ElmoECAT::SetOperationMode(uint8_t om)
{
    if( ecrt_slave_config_sdo8(slaveConfig,od_operationMode, om) )
        std::cout << "Set operation mode config error ! " << std::endl;
}

int ElmoECAT::GetProfilePositionParameters (ProfilePosParam& P, sdoRequest_t& sr)
{
    ReadSDO(sr.s_ProfileAcceleration, P.profileAcceleration);
}

int ElmoECAT::SetProfilePositionParameters( ProfilePosParam& P ) 
{
    // Returns 0 if succesfull, otherwise < 0 
    //profile velocity
    if(ecrt_slave_config_sdo32(slaveConfig,od_profileVelocity, P.profileVelocity) < 0) {
        std::cout << "Set profile velocity config error ! " << std::endl;
        return -1;
    }
    //max profile velocity
    if(ecrt_slave_config_sdo32(slaveConfig,od_maxProfileVelocity,P.maxProfileVelocity) < 0) {
        std::cout << "Set max profile  velocity config error ! " << std::endl;
        return -1;
    }
    //profile acceleration
    if(ecrt_slave_config_sdo32(slaveConfig,od_profileAcceleration, P.profileAcceleration) < 0) {
        std::cout << "Set profile acceleration failed ! " << std::endl;
        return -1;
    }
    //profile deceleration
    if(ecrt_slave_config_sdo32(slaveConfig,od_profileDeceleration,P.profileDeceleration) < 0) {
        std::cout << "Set profile deceleration failed ! " << std::endl;
        return -1;
    }
    // quick stop deceleration 
    if(ecrt_slave_config_sdo32(slaveConfig,od_quickStopDeceleration,P.quickStopDeceleration) < 0) {
        std::cout << "Set profile deceleration failed ! " << std::endl;
        return -1;
    }
    // max following error 
    if(ecrt_slave_config_sdo16(slaveConfig,od_maxFollowingError,P.maxFollowingError) < 0) {
        std::cout << "Set profile deceleration failed ! " << std::endl;
        return -1;
    }   
    return 0;
}

int ElmoECAT::SetProfileVelocityParameters(ProfileVelocityParam& P)
{
    // Returns 0 if succesfull, otherwise < 0 
    // motionProfileType
    if(ecrt_slave_config_sdo16(slaveConfig,od_motionProfileType, P.MotionProfileType) < 0) {
        std::cout << "Set profile velocity config error ! " << std::endl;
        return -1;
    }
    //max profile velocity
    if(ecrt_slave_config_sdo32(slaveConfig,od_maxProfileVelocity,P.MaxProfileVelocity) < 0) {
        std::cout << "Set max profile  velocity config error ! " << std::endl;
        return -1;
    }
    //profile acceleration
    if(ecrt_slave_config_sdo32(slaveConfig,od_profileAcceleration, P.ProfileDecel) < 0) {
        std::cout << "Set profile acceleration failed ! " << std::endl;
        return -1;
    }
    //profile deceleration
    if(ecrt_slave_config_sdo32(slaveConfig,od_profileDeceleration,P.ProfileAccel) < 0) {
        std::cout << "Set profile deceleration failed ! " << std::endl;
        return -1;
    }
    // quick stop deceleration 
    if(ecrt_slave_config_sdo32(slaveConfig,od_quickStopDeceleration,P.QuickStopDecel) < 0) {
        std::cout << "Set profile deceleration failed ! " << std::endl;
        return -1;
    }
    return 0 ; 
}


int ElmoECAT::ActivateMaster()
{
    // Before activating master all configuration should be done \
    activating master means now you're ready for realtime PDO data exchange
    if ( ecrt_master_activate(master) ) {
        std::cout << " Master activation error ! " << std::endl;
        return -1;
    }
    if(!(slavePdoDomain = ecrt_domain_data(masterDomain)))
    {
        std::cout << "Domain PDO registration error ... " << std::endl;
        return -1;
    }
}
/*
void ElmoECAT::CheckSlaveConfigurationState()
{
    ec_slave_config_state_t s;
    ecrt_slave_config_state(slaveConfig, &s);

    if (slaves_up < NUM_OF_SLAVES && s.al_state != 0x08) {
        printf("Gold Solo Slave  : State 0x%02X.\n", s.al_state);
    }
    if (slaves_up < NUM_OF_SLAVES && s.al_state == 0x08) {
          printf("Gold Solo Slave : State 0x%02X.\n", s.al_state);
        slaves_up = NUM_OF_SLAVES;
    }
    if (s.al_state != slaveConfig_state.al_state) {
        printf("AnaIn: State 0x%02X.\n", s.al_state);
    }
    if (s.online != slaveConfig_state.online) {
        printf("AnaIn: %s.\n", s.online ? "online" : "offline");
    }
    if (s.operational != slaveConfig_state.operational) {
        printf("AnaIn: %soperational.\n", s.operational ? "" : "Not ");
    }
    
     slaveConfigState = s;
}

int ElmoECAT::CheckMasterState()
{
    ec_master_state_t ms;
    ecrt_master_state(master, &ms);

    if (ms.slaves_responding != masterState.slaves_responding){
        printf("%u slave(s).\n", ms.slaves_responding);

        if (ms.slaves_responding < 1) {
            printf("Connection error, only %d slaves responding",ms.slaves_responding);
            return 0;
        }
    }
    if (ms.al_states != masterState.al_states){
        printf("AL states: 0x%02X.\n", ms.al_states);
    }
    if (ms.link_up != masterState.link_up){
        printf("Link is %s.\n", ms.link_up ? "up" : "down");
        if(!ms.link_up) 
        return 0;
    }
    masterState = ms;
    return 1;
}

void ElmoECAT::CheckMasterDomainState()
{
    ec_domain_state_t ds;                     //Domain instance
    ecrt_domain_state(masterDomain, &ds);

    if (ds.working_counter != masterDomainState.working_counter)
        printf("masterDomain: WC %u.\n", ds.working_counter);
    if (ds.wc_state != masterDomainState.wc_state)
        printf("masterDomain: State %u.\n", ds.wc_state);
    if(masterDomainState.wc_state == EC_WC_COMPLETE){
        printf("All slaves configured...\n");
        masterDomainState = ds;
    }
    masterDomainState = ds;
}
*/
void ElmoECAT::WaitForOPmode()
{
    while (slaves_up != NUM_OF_SLAVES){
        ecrt_master_receive(master);
        ecrt_domain_process(masterDomain);
        usleep(500);

       /* CheckMasterState();
        CheckMasterDomainState();
        CheckSlaveConfigurationState();
        */
        clock_gettime(CLOCK_TO_USE, &syncTimer);
        ecrt_master_sync_reference_clock_to(master, TIMESPEC2NS(syncTimer));
        ecrt_master_sync_slave_clocks(master);
        ecrt_master_application_time(master, TIMESPEC2NS(syncTimer));

        ecrt_domain_queue(masterDomain);                
        ecrt_master_send(master);
        usleep(500);
    }
}

void ElmoECAT::ResetMaster()
{
    ecrt_master_reset(master);
}

/*
int ElmoECAT::Enable(uint8_t *domain1_pd_)
{
    // read process data
    data.actual_pos     = EC_READ_S32(domain1_pd_ + offset.actual_pos);
    data.actual_vel     = EC_READ_S32(domain1_pd_ + offset.actual_vel);
    data.actual_cur     = EC_READ_S16(domain1_pd_ + offset.actual_cur);
    data.actual_tor     = EC_READ_S16(domain1_pd_ + offset.actual_tor);
    data.status_word    = EC_READ_U16(domain1_pd_ + offset.status_word);
    data.mode_display   = EC_READ_U8(domain1_pd_ + offset.mode_display);
    int motor_status = data.status_word & 0x006f;

    if(data.mode_display != 0x06)
    {
        std::cout << "--------mode_display--------" << std::endl;
        EC_WRITE_S8(domain1_pd_ + offset.mode, static_cast<int8_t>(MODE_PROFILE_POSITION));
    } else {
        //motor enable
        if(motor_status == 0x0040 || motor_status == 0x0600)// switch on disable
        {
            EC_WRITE_U16(domain1_pd_ + offset.control_word, 0x0006); //shut down
            std::cout << "--------shut down--------" << std::endl;
        }
        else if (motor_status == 0x0021) //read to switch on
        {
            EC_WRITE_U16(domain1_pd_ + offset.control_word, 0x0007); //switch on
            std::cout << "-------switch on---------" << std::endl;
        }
        else if (motor_status == 0x0023) //switch on
        {
            EC_WRITE_U16(domain1_pd_ + offset.control_word, 0x000f); //Enable Operation
            std::cout << "-------Enable Operation---------" << std::endl;
        }
        else if(motor_status == 0x0027)//operation enable
        {
            //successfull, but still need to wait for 10 more cycles to make it stable
            static int enable_period_ = 0;
            if(++enable_period_ > 20)
            {
                enable_period_ = 0;
                motor_state = STATE_ENABLED;
                std::cout << "motor has been enabled" << std::endl;
                std::cout << "--------operation enable--------" << std::endl;
            }

        }
        else //fault
        {
            std::cout << "---------FAULT reset-------" << std::endl;

            EC_WRITE_U16(domain1_pd_ + offset.control_word, 0x0080); //FAULT reset
            static int counter = 0;
            if(++counter > 500)
            {
                counter = 0;
                motor_state = STATE_FAULT;
            }
        }
    }

    return 0;
}

int ElmoECAT::ProfilePositionMode(uint8_t *domain1_pd_)
{
    data.status_word  = EC_READ_U16(domain1_pd_ + offset.status_word);
    data.mode_display = EC_READ_U8(domain1_pd_ + offset.mode_display);

    int motor_status = data.status_word & 0x006f;

    //motor not enabled
    if(motor_status != 0x0027){        
        std::cout << data.status_word << std::endl;
        std::cout << "Profile position mode error, please enable motor first!" << std::endl;
        return -1;
    }else { //motor has enabled
    
        //set homeing mode
        if(data.mode_display != MODE_PROFILE_POSITION)
        {
            std::cout << "--------mode_display--------" << std::endl;
            EC_WRITE_S8(domain1_pd_ + offset.mode, static_cast<int8_t>(MODE_PROFILE_POSITION));
        } else {

            if((data.status_word & 0x3400) == 0x0400)//homing procedure is interrupted or not started
            {
                std::cout << "to start homing procedure" << std::endl;
                EC_WRITE_U16(domain1_pd_ + offset.control_word, static_cast<uint16_t>(0x1f));

            }
            else if((data.status_word & 0x3400) == 0x0000) //homing ing
            {
                std::cout << "homing procedure is running................."<< std::endl;
            }
            else if((data.status_word & 0x3400) == 0x1400) //homing ing
            {
                std::cout << "homing successfully"<< std::endl;
                motor_state =STATE_HOMED;
            }
        }
    }
}

int ElmoECAT::SetMode(uint8_t *domain1_pd_, int8_t mode_)
{
    data.actual_pos     = EC_READ_S32(domain1_pd_ + offset.actual_pos);
    data.actual_vel     = EC_READ_S32(domain1_pd_ + offset.actual_vel);
    data.actual_cur     = EC_READ_S16(domain1_pd_ + offset.actual_cur);
    data.actual_tor     = EC_READ_S16(domain1_pd_ + offset.actual_tor);
    data.status_word    = EC_READ_U16(domain1_pd_ + offset.status_word);
    data.mode_display   = EC_READ_U8 (domain1_pd_ + offset.mode_display);


    int motor_status = data.status_word & 0x006f;
    if(motor_status != 0x0027)
    {
        std::cout << " Operation Mode error, please enable motor first!" << std::endl;
        return -1;
    }
    else
    {
        switch (mode_)
        {
            case MODE_PROFILE_POSITION:
            case MODE_CSP:
                //targetposition should be equal to actualposition
                data.actual_pos = EC_READ_S32(domain1_pd_ + offset.actual_pos);
                EC_WRITE_S32(domain1_pd_ + offset.target_pos, static_cast<int32_t>(data.actual_pos));
                motor_state = STATE_CSP;
                break;
            case MODE_CSV:
                //velocity loop to set velocity of 0
                EC_WRITE_S32(domain1_pd_ + offset.target_vel, static_cast<int32_t>(0));
                motor_state = STATE_CSV;
                break;
            case MODE_CST:
                EC_WRITE_S16(domain1_pd_ + offset.target_tor, static_cast<int32_t>(0));
                motor_state = STATE_CST;
                break;
            default:
                return -1;
                break;
        }
    }

    if(data.mode_display == mode_)
    {
        std::cout << "mode set successful";
        return 0;
    }
    else
    {
        EC_WRITE_S8(domain1_pd_ + offset.mode, static_cast<int8_t>(mode_));
    }
    return 0;
}

int ElmoECAT::SetTargtVelocity(uint8_t *domain1_pd_, int32_t velocity_)
{
    if(motor_state != STATE_CSV)
    {
        return -1;
    }

    EC_WRITE_S32(domain1_pd_ + offset.target_vel, velocity_);
}

int ElmoECAT::Display(uint8_t *domain1_pd_)
{
    data.actual_pos   = EC_READ_S32(domain1_pd_ + offset.actual_pos);
    data.actual_vel   = EC_READ_S32(domain1_pd_ + offset.actual_vel);
    data.actual_cur   = EC_READ_S16(domain1_pd_ + offset.actual_cur);
    data.actual_tor   = EC_READ_S16(domain1_pd_ + offset.actual_tor);
    data.status_word  = EC_READ_U16(domain1_pd_ + offset.status_word);
    data.mode_display = EC_READ_U8(domain1_pd_  + offset.mode_display);


    std::cout << "actual_pos:" << std::dec << data.actual_pos << std::endl;
    std::cout << "actual_vel:" << std::dec << data.actual_vel << std::endl;
    std::cout << "actual_cur:" << std::dec << data.actual_cur << std::endl;
    std::cout << "actual_tor:" << std::dec << data.actual_tor << std::endl;
    std::cout << "status_word: 0x" << std::hex << data.status_word << std::endl;
    std::cout << "mode_display: 0x" << std::hex << (uint16_t)data.mode_display << std::endl;
    std::cout << std::endl;
    return 0;
}
*/