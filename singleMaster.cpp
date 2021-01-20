#include <iostream>


class EcatMaster
{
private:
    /* Private constructor to prevent instancing-> */
    EcatMaster();

public:
    /* Static access method-> */
    static EcatMaster *getInstance();
    int a = 5 ;
    int b = 3 ; 
    void func()
    {
        
        a  = 2 ; 
    }
};

EcatMaster *EcatMaster::getInstance()
{
    static EcatMaster instance;

    return &instance;
}

EcatMaster::EcatMaster()
{
}
class foo 
{
    public :
    EcatMaster *smaster = EcatMaster::getInstance();


};
int main()
{
    foo A, B ;
    //new EcatMaster(); // Won't work
    EcatMaster *master = EcatMaster::getInstance() ;
    EcatMaster *r = EcatMaster::getInstance() ;


    /* The addresses will be the same-> */
    std::cout << master->a << std::endl;
    std::cout << r->a << std::endl;
    std::cout << A.smaster->a << std::endl;

    master -> a = 55 ;
    A.smaster->func();
    std::cout << master->a << std::endl;
    std::cout << r->a << std::endl;
    std::cout << A.smaster->a << std::endl;

    A.smaster -> a = 33 ;
    std::cout << master->a << std::endl;
    std::cout << r->a << std::endl;
    std::cout << A.smaster->a << std::endl;
}
