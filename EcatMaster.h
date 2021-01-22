#include "globals.h"


// Singleton pattern for master . 

class EcatMaster
{
private:
    /* Private constructor to prevent instancing-> */
    EcatMaster();

public:
    /* Static access method-> */
    EcatMaster *getInstance();
    ec_master_t           *master = NULL ;
    ec_master_state_t      masterState = {};
    ec_domain_t           *masterDomain = NULL; 
    ec_domain_state_t      masterDomainState = {};  
    char                   slaves_up = 0 ;
    struct timespec        syncTimer ;
    void foo();
    void bar();
};