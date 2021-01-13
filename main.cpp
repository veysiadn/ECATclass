#include "ElmoECAT.h"

#define PERIODNS  1e6

int main()
{
    ElmoECAT e_motor;
    e_motor.cycleTime    = PERIODNS;
    e_motor.sync0_shift  = 0;
    std::cout << "Initialization started.." << std::endl;
   /* if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
    //mlockall locks part of all the calling process's \ virtual address space into RAM, \
     preventing that memory being paged to swap area.To prevent realtime performance drops.
        std::cout << "mlockall failed" << std::endl;
        return -1;
    }*/
    
    e_motor.ConfigureMaster();
    std::cout << "Initialization  2 started.." << std::endl;
    e_motor.position_      = 0 ;
    e_motor.ConfigureSlave(e_motor.position_);
    std::cout << "Initialization  3 started.." << std::endl;
    //e_motor.MapPDOs();
    e_motor.ConfigDCSync();
    std::cout << "Initialization 4 started.." << std::endl;
    ProfilePosParam PositionParameters;
    PositionParameters.maxProfileVelocity    = 1e5 ;
    PositionParameters.profileAcceleration   = 1e6 ;
    PositionParameters.profileDeceleration   = 1e6 ;
    PositionParameters.profileVelocity       = 8e4 ;
    PositionParameters.quickStopDeceleration = 1e6 ;
    PositionParameters.maxFollowingError     = 1e6 ; 
    e_motor.SetProfilePositionParameters(PositionParameters);
    std::cout << "Initialization 5 started.." << std::endl; 
    e_motor.ActivateMaster();
    std::cout << "Initialization 6 started.." << std::endl;     
    return 0;
}