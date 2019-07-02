#ifndef EPOS_FUNCTIONS_H
#define EPOS_FUNCTIONS_H

#include <std_msgs/String.h>
#include <ros/ros.h>
#include <epos_library/Definitions.h>
#include <cmath>

//Unidades de maxon:
//  Posición    [grados]
//  Velocidad   [rpm]
//  Aceleración [rpm/s]

class epos_functions
{

private:



public:
    epos_functions();
    ~epos_functions();
    void SetDefaultParameters(int ); //Creo que esta no hace falta para nada

    // Abrir Epos
    //int OpenDevice(unsigned int* , int , void* , std::string , std::string , std::string , std::string , int );
    int OpenDevice(unsigned int* errorCode, int node_id);
    // Eliminar fallos
    int ClearErrors(unsigned int* p_pErrorCode);

    void LogError(std::string functionName, int p_lResult, unsigned int p_ulErrorCode);
    void LogInfo(std::string message);

    int ActivateProfilePosition(int motor);
    int SetPositionProfile(int motor, double velocity, double acceleration, double deceleration);
    int MoveToPosition(int motor, double position, bool absolute, bool inmediately);
    double GetPosition(int motor);
    bool HaltPositionMovement(int motor);

    int ActivateProfileVelocity(int motor);
    int SetVelocityProfile(int motor, double acceleration, double deceleration);
    int MoveWithVelocity(int motor, double velocity);
    double GetVelocity(int motor);
    bool HaltVelocityMovement(int motor);

    int GetEffort(int);

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
