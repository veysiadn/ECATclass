#include "EcatMaster.h"

EcatMaster *EcatMaster::getInstance()
{
    static EcatMaster instance;

    return &instance;
}

EcatMaster::EcatMaster()
{
}