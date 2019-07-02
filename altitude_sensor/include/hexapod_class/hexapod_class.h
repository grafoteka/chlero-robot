#ifndef HEXAPOD_CLASS_H
#define HEXAPOD_CLASS_H

#include <altitude_sensor/Definitions.h>
#include <std_msgs/String.h>
#include <string>
#include <ros/ros.h>

class hexapod_class
{

private:

    typedef void* HANDLE;

    void* g_pKeyHandle_ = 0;
    unsigned short g_usNodeId_;
    std::string g_deviceName_;
    std::string g_protocolStackName_;
    std::string g_interfaceName_;
    std::string g_portName_;
    int g_baudrate_ = 0;

    const std::string g_programName = "HelloEposCmd";

    #ifndef MMC_SUCCESS
        #define MMC_SUCCESS 0
    #endif

    #ifndef MMC_FAILED
        #define MMC_FAILED 1
    #endif

    #ifndef MMC_MAX_LOG_MSG_SIZE
        #define MMC_MAX_LOG_MSG_SIZE 512
    #endif

    void LogError(std::string functionName, int p_lResult, unsigned int p_ulErrorCode);
    void LogInfo(std::string message);
    void SeparatorLine();
    void PrintSettings();
    void SetDefaultParameters();
    int  OpenDevice(unsigned int* p_pErrorCode);
    int  PrepareEpos(unsigned int* p_pErrorCode);
    void move_legs();



public:

    hexapod_class();    // Constructor

    int epos_setup();

    bool stand_up(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode);

    void lay_down();




};

#endif // HEXAPOD_CLASS_H
