#include "ElmoECAT.h"

void* Start_Profile_Position_Cyclic_Task(void *arg)
{

}
void* Start_Profile_Velocity_Cyclic_Task(void *arg)
{

}
void* Start_Controller_Read_Task(void * arg)
{

}


int main()
{

    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
    //mlockall locks part of all the calling process's \ virtual address space into RAM, \
     preventing that memory being paged to swap area.To prevent realtime performance drops.
        std::cout << "mlockall failed" << std::endl;
        return -1 ;
    }
//----------------------------------------------------------------------------//
    // Constructor for ElmoECAT class will handle the initial configuration for \
        individual slaves.In case of any error check ElmoECAT constructor.
    ElmoECAT e_motor[NUM_OF_SLAVES];
//--------------------------------------------------------------------------------//
   // For position mode just change the PrepareForProfileVelocityMode() function to \
        PrepareForProfilePosition() function. This will initialize motors with \
        default parameters. If you want to change the parameters you should reimplement \
        prepare functions.
    for(int i = 0 ; i < NUM_OF_SLAVES ; i++){
        if( e_motor[i].PrepareForProfileVelocityMode() )
            return -1 ;
    }

//--------------------------------------------------------------------------------//
// Activation of master can be done by one slave since there's only one master.
    if (e_motor[0].ActivateMaster())
        return -1 ;
//--------------------------------------------------------------------------------//
    for(int i = 0 ; i < NUM_OF_SLAVES ; i++){
        if (e_motor[i].RegisterDomain())
            return -1 ;
    }
    if ( e_motor.WaitForOPmode() )
        return -1;
//--------------------------------------------------------------------------------//
    std::cout << "Motor in OP Mode " << std::endl ; 
    
    e_motor.StartRealTimeTasks(e_motor);
        std::cout << " Realtime task started ... " << std::endl ;
    e_motor.KeepInOPmode();
    return 0;
}