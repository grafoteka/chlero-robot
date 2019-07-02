#ifndef EPOS_FUNCTIONS_H
#define EPOS_FUNCTIONS_H

#include <std_msgs/String.h>
#include <ros/ros.h>
#include <maxon_library/Definitions.h>

class epos_functions
{

private:



public:
    epos_functions();
    void SetDefaultParameters(int ); //Creo que esta no hace falta para nada

    // Abrir Epos
    //int OpenDevice(unsigned int* , int , void* , std::string , std::string , std::string , std::string , int );
    int OpenDevice(unsigned int*, int);
    // Eliminar fallos
    int ClearErrors(unsigned int* );

    void LogError(std::string , int , unsigned int );
    void LogInfo(std::string );

    int ActivateProfilePosition(int);
    int SetPositionProfile(int, int, int, int);
    int MoveToPosition(int, int, bool, bool);
    int GetPosition(int);
    bool HaltPositionMovement(int);

    //void epos_functions::move_motor_to_position(void, unsigned short, long, unsigned int, unsigned int, unsigned int, unsigned int, bool, bool )
    //void move_motor_to_position(void*, unsigned short, long, unsigned int, unsigned int, unsigned int, unsigned int, bool, bool);

    // Variables
    #ifndef MMC_SUCCESS
        #define MMC_SUCCESS 0
    #endif

    #ifndef MMC_FAILED
        #define MMC_FAILED 1
    #endif

    #ifndef MMC_MAX_LOG_MSG_SIZE
        #define MMC_MAX_LOG_MSG_SIZE 512
    #endif

    unsigned int ulErrorCode;
    void* keyHandle;
    unsigned short node_id;
    std::string deviceName;// = "EPOS";
    std::string protocolStackName ;//= "MAXON_RS232";
    std::string interfaceName ;//= "RS_232";
    std::string portName ;//= "/dev/ttyS1";
    int baudrate ;//= 115200;

};

#endif // EPOS_FUNCTIONS_H
