#include "hexapod_class.h"

void hexapod_class::move_legs()
{

}

bool hexapod_class::stand_up(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode)
{
    int lResult = MMC_SUCCESS;

    const double tripod_one_position = 345; //330º
    const double tripod_two_position = 375;  // Esto debe ser 360º + 15º

    // Esta es la cantidad de grados que se deben de mover las patas desde el estado inicial que corresponde a 4.57 radianes
    const double offset_inicial = 260;

    const double take_off_angle = 15;

    static double leg_actual_pos[6];
    static double leg_actual_error[6];
    static double leg_actual_error_old[6];
    static double leg_desired_position[6] = {offset_inicial, (offset_inicial + take_off_angle), (offset_inicial + take_off_angle), offset_inicial, offset_inicial, (offset_inicial + take_off_angle)};

    const int tripod_one[3] = {1, 4, 5};
    const int tripod_two[3] = {2, 3, 6};

    static bool pose = false;

    static bool leg_reached_offset[6] = {false, false, false, false, false, false};

    std::cout << "Desired position: " << offset_inicial << std::endl;

    // Activar el modo de velocidad
    for(int i = 1; i <= 6; i++)
    {
        p_usNodeId = i;
        if(VCS_ActivateProfileVelocityMode(p_DeviceHandle, p_usNodeId, &p_rlErrorCode) == 0)
        {
            LogError("VCS_ActivateProfileVelocityMode", lResult, p_rlErrorCode);
            lResult = MMC_FAILED;
        }
    }

    // Movimiento de las patas al offset
    while(!pose)
    {
        for(int i = 1; i <= 6; i++)
        {
            int leg_actual_pos_raw;
            int p_usNodeId = i;
            VCS_GetPositionIs(p_DeviceHandle, p_usNodeId, &leg_actual_pos_raw, &p_rlErrorCode);
            leg_actual_pos[i-1] = fabs((leg_actual_pos_raw / 2000.0) * 2 * M_PI);
            leg_actual_error[i-1] = fabs(leg_desired_position[i-1] - leg_actual_pos[i-1]);

            // Muestra el error de posicion si la pata tiene que moverse
            if(leg_actual_error_old[i-1] != leg_actual_error[i-1])
            {
                std::stringstream msg;
                msg << "leg = " << p_usNodeId << ", position = " << leg_actual_pos[i-1] << ", error = " << leg_actual_error[i-1] << std::endl;
                LogInfo(msg.str());
            }

            // Si el error de consigna es menor a 3 grados se da como buena la posicion
            if(leg_actual_error[i-1] <= 4)
            {
                leg_reached_offset[i-1] = true;
                VCS_HaltVelocityMovement(p_DeviceHandle, p_usNodeId, &p_rlErrorCode);
            }

            if(!leg_reached_offset[i-1])
            {
                double velocity = 100;

                if((i == 2) || (i == 4) || (i == 6))
                {
                    velocity = -velocity;
                }

                else
                    velocity = velocity;

                if(VCS_MoveWithVelocity(p_DeviceHandle, p_usNodeId, velocity, &p_rlErrorCode) == 0)
                {
                    lResult = MMC_FAILED;
                    LogError("VCS_MoveWithVelocity", lResult, p_rlErrorCode);
                    break;
                }
            }

            leg_actual_error_old[i] = leg_actual_error[i];
        }

        if(leg_reached_offset[0] && leg_reached_offset[1] && leg_reached_offset[2] && leg_reached_offset[3] && leg_reached_offset[4] && leg_reached_offset[5])
        {
            pose = true;
            std::stringstream msg;
            msg << "Pose = true" << std::endl;
            LogInfo(msg.str());
        }
    }

    return true;
}

void hexapod_class::lay_down()
{

}


int hexapod_class::epos_setup()
{
    int lResult = MMC_SUCCESS;
    unsigned int ulErrorCode = 0;

    static bool epos_setup_flag = false;

    this->SetDefaultParameters();

    this->PrintSettings();

    // First Open the device connection
    if((lResult = OpenDevice(&ulErrorCode))!=MMC_SUCCESS)
    {
            LogError("OpenDevice", lResult, ulErrorCode);
            return lResult;
    }

    if((lResult = this->PrepareEpos(&ulErrorCode))!=MMC_SUCCESS)
    {
            LogError("PrepareEpos", lResult, ulErrorCode);
            return lResult;
    }

    std::cout << "EPOS has been initialize" << std::endl;

}



void hexapod_class::LogError(std::string functionName, int p_lResult, unsigned int p_ulErrorCode)
{
    std::cerr << g_programName << ": " << functionName << " failed (result=" << p_lResult << ", errorCode=0x" << std::hex << p_ulErrorCode << ")"<< std::endl;
}

void hexapod_class::LogInfo(std::string message)
{
    std::cout << message << std::endl;
}

void hexapod_class::PrintSettings()
{
    std::stringstream msg;

    msg << "default settings:" << std::endl;
    msg << "node id             = " << g_usNodeId_ << std::endl;
    msg << "device name         = '" << g_deviceName_ << "'" << std::endl;
    msg << "protocal stack name = '" << g_protocolStackName_ << "'" << std::endl;
    msg << "interface name      = '" << g_interfaceName_ << "'" << std::endl;
    msg << "port name           = '" << g_portName_ << "'"<< std::endl;
    msg << "baudrate            = " << g_baudrate_;

    LogInfo(msg.str());

    SeparatorLine();
}

void hexapod_class::SetDefaultParameters()
{
    //RS232
        g_usNodeId_ = 5;
        g_deviceName_ = "EPOS";

        g_protocolStackName_ = "MAXON_RS232";
        g_interfaceName_ = "RS232";
        g_portName_ = "/dev/ttyS1";
        g_baudrate_ = 115200;

    // CAN
        //g_protocolStackName = "CANopen";
        //g_interfaceName = "CAN_ixx_usb 0";
        //g_portName = "CAN0";
        //g_baudrate = 1000000;
}

// Function to connect to the Epos
int hexapod_class::OpenDevice(unsigned int *p_pErrorCode)
{
    int lResult = MMC_FAILED;

    char* pDeviceName = new char[255];
    char* pProtocolStackName = new char[255];
    char* pInterfaceName = new char[255];
    char* pPortName = new char[255];

    strcpy(pDeviceName, g_deviceName_.c_str());
    strcpy(pProtocolStackName, g_protocolStackName_.c_str());
    strcpy(pInterfaceName, g_interfaceName_.c_str());
    strcpy(pPortName, g_portName_.c_str());

    LogInfo("Open device...");

    g_pKeyHandle_ = VCS_OpenDevice(pDeviceName, pProtocolStackName, pInterfaceName, pPortName, p_pErrorCode);

    if(g_pKeyHandle_!=0 && *p_pErrorCode == 0)
    {
        unsigned int lBaudrate = 0;
        unsigned int lTimeout = 0;

        if(VCS_GetProtocolStackSettings(g_pKeyHandle_, &lBaudrate, &lTimeout, p_pErrorCode)!=0)
        {
            if(VCS_SetProtocolStackSettings(g_pKeyHandle_, g_baudrate_, lTimeout, p_pErrorCode)!=0)
            {
                if(VCS_GetProtocolStackSettings(g_pKeyHandle_, &lBaudrate, &lTimeout, p_pErrorCode)!=0)
                {
                    if(g_baudrate_==(int)lBaudrate)
                    {
                        lResult = MMC_SUCCESS;
                    }
                }
            }
        }
    }
    else
    {
        g_pKeyHandle_ = 0;
    }

    delete []pDeviceName;
    delete []pProtocolStackName;
    delete []pInterfaceName;
    delete []pPortName;

        if(lResult == MMC_SUCCESS)
        {
            LogInfo("Device openned correctly");
        }

    return lResult;
}

// Function to get the errors of the EPOs, clear the errors and set the Enable State
int hexapod_class::PrepareEpos(unsigned int *p_pErrorCode)
{
    std::cout << "Reading EPOS error" << std::endl;

    int lResult = MMC_SUCCESS;
    int oIsFault = 0;

        for(int i = 1; i <= 6; i++)
        {
            g_usNodeId_ = i;

            if(VCS_GetFaultState(g_pKeyHandle_, g_usNodeId_, &oIsFault, p_pErrorCode ) == 0)
            {
                    LogError("VCS_GetFaultState", lResult, *p_pErrorCode);
                    lResult = MMC_FAILED;
            }

            if(lResult==0)
            {
                    if(oIsFault)
                    {
                            std::stringstream msg;
                            msg << "clear fault, node = '" << g_usNodeId_ << "'";
                            LogInfo(msg.str());

                            if(VCS_ClearFault(g_pKeyHandle_, g_usNodeId_, p_pErrorCode) == 0)
                            {
                                    LogError("VCS_ClearFault", lResult, *p_pErrorCode);
                                    lResult = MMC_FAILED;
                            }
                    }

                    if(lResult==0)
                    {
                            int oIsEnabled = 0;

                            if(VCS_GetEnableState(g_pKeyHandle_, g_usNodeId_, &oIsEnabled, p_pErrorCode) == 0)
                            {
                                    LogError("VCS_GetEnableState", lResult, *p_pErrorCode);
                                    lResult = MMC_FAILED;
                            }

                            if(lResult==0)
                            {
                                    if(!oIsEnabled)
                                    {
                                            if(VCS_SetEnableState(g_pKeyHandle_, g_usNodeId_, p_pErrorCode) == 0)
                                            {
                                                    LogError("VCS_SetEnableState", lResult, *p_pErrorCode);
                                                    lResult = MMC_FAILED;
                                            }
                                    }
                            }
                    }
            }
        }
    return lResult;
}

void hexapod_class::SeparatorLine()
{
    const int lineLength = 65;
    for(int i=0; i<lineLength; i++)
    {
        std::cout << "-";
    }
    std::cout << std::endl;
}


/******************************************************
 * Constructor
 ******************************************************/
hexapod_class::hexapod_class()
{
    ROS_INFO("STAND BY Constructor creado");
}
