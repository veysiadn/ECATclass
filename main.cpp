#include "ElmoECAT.h"

#define PERIODNS  1e6

int main()
{
    ElmoECAT e_motor;
    e_motor.cycleTime    = PERIODNS;
    e_motor.sync0_shift  = 0;
    e_motor.position_ = 0 ;

    std::cout << "Initialization started.." << std::endl;
    
   if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
    //mlockall locks part of all the calling process's \ virtual address space into RAM, \
     preventing that memory being paged to swap area.To prevent realtime performance drops.
        std::cout << "mlockall failed" << std::endl;
        return -1 ;
    }

    if ( e_motor.ConfigureMaster() ){
        std::cout << "Master configuration error..." << std::endl;
        return -1 ;
    }

    if ( e_motor.ConfigureSlave(e_motor.position_) ){
        std::cout << "Slave configuration error..." << std::endl;
        return 1 ;
    }
    if ( e_motor.MapPDOs(e_motor.GS_Syncs,e_motor.masterDomain_PdoRegs) ){
        std::cout << "PDO registration error..." << std::endl;
        return -1 ;
    }
    e_motor.ConfigDCSync();
    std::cout << "DC Sync method configured.." << std::endl;
    ProfilePosParam PositionParameters; 
    // sdoRequest_t e_sdo;
    e_motor.GetDefaultPositionParameters(PositionParameters);

    e_motor.SetOperationMode(MODE_PROFILE_POSITION);
    if ( e_motor.SetProfilePositionParameters(PositionParameters)  ){
        std::cout << "Profile Position Parameter set error.." << std::endl; 
        return -1 ;
    }
    if (e_motor.ActivateMaster()){
        std::cout << "Master Activation error.." << std::endl;  
        return -1 ;
    }
    if ( e_motor.WaitForOPmode() ){
        std::cout << "Error while waiting for OP mode..." << std::endl;
        return -1;
    }
    e_motor.StartRealTimeTasks();
    
    return 0;
}