#include <ros/ros.h>
#include <std_msgs/String.h>
#include <altitude_sensor/sensor_data.h>
#include <cereal_port/CerealPort.h>
#include <altitude_sensor/Definitions.h>
#include <sensor_msgs/JointState.h>

ros::Publisher joint_states_pub;

#define REPLY_SIZE 20
#define TIMEOUT 1000

const unsigned int sensor_frequency = 10; /*sensor frequency in hz*/

double altitude;
double voltage;
std::string serial_port;

typedef void* HANDLE;
typedef int BOOL;

using namespace std;

void* g_pKeyHandle = 0;
unsigned short g_usNodeId;
string g_deviceName;
string g_protocolStackName;
string g_interfaceName;
string g_portName;
int g_baudrate = 0;
//    EAppMode g_eAppMode = AM_DEMO;

// Vector para guardar el offset inicial de las patas.
double motor_initial_offset_[6];

const string g_programName = "HelloEposCmd";

#ifndef MMC_SUCCESS
    #define MMC_SUCCESS 0
#endif

#ifndef MMC_FAILED
    #define MMC_FAILED 1
#endif

#ifndef MMC_MAX_LOG_MSG_SIZE
    #define MMC_MAX_LOG_MSG_SIZE 512
#endif

void  LogError(string functionName, int p_lResult, unsigned int p_ulErrorCode);
void  LogInfo(string message);
void  PrintSettings();
void  SetDefaultParameters();
int   OpenDevice(unsigned int* p_pErrorCode);
int   PrepareDemo(unsigned int* p_pErrorCode);
int   Demo(unsigned int* p_pErrorCode);
int   stand_up(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode);
int   prepare_legs(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode);
int   altern_tripod(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode);
void joint_state_publish(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode);

void LogError(string functionName, int p_lResult, unsigned int p_ulErrorCode)
{
    cerr << g_programName << ": " << functionName << " failed (result=" << p_lResult << ", errorCode=0x" << std::hex << p_ulErrorCode << ")"<< endl;
}

void LogInfo(string message)
{
    cout << message << endl;
}

void SeparatorLine()
{
    const int lineLength = 65;
    for(int i=0; i<lineLength; i++)
    {
        cout << "-";
    }
    cout << endl;
}

void PrintSettings()
{
    stringstream msg;

    msg << "default settings:" << endl;
    msg << "node id             = " << g_usNodeId << endl;
    msg << "device name         = '" << g_deviceName << "'" << endl;
    msg << "protocal stack name = '" << g_protocolStackName << "'" << endl;
    msg << "interface name      = '" << g_interfaceName << "'" << endl;
    msg << "port name           = '" << g_portName << "'"<< endl;
    msg << "baudrate            = " << g_baudrate;

    LogInfo(msg.str());

    SeparatorLine();
}

//void SetDefaultParameters()
void SetDefaultParameters(int node_id)
{
    //USB
        g_usNodeId = node_id;//1;
        g_deviceName = "EPOS";
        //g_protocolStackName = "CANopen";
        g_protocolStackName = "MAXON_RS232";
        //g_interfaceName = "CAN_ixx_usb 0";
        g_interfaceName = "RS232";
        //g_portName = "CAN0";
        g_portName = "/dev/ttyS1";
        //g_baudrate = 1000000;
        g_baudrate = 115200;
}

int OpenDevice(unsigned int* p_pErrorCode, int node_id)
{
    int lResult = MMC_FAILED;

    char* pDeviceName = new char[255];
    char* pProtocolStackName = new char[255];
    char* pInterfaceName = new char[255];
    char* pPortName = new char[255];

    strcpy(pDeviceName, g_deviceName.c_str());
    strcpy(pProtocolStackName, g_protocolStackName.c_str());
    strcpy(pInterfaceName, g_interfaceName.c_str());
    strcpy(pPortName, g_portName.c_str());

    //LogInfo("Open device...");
    ROS_INFO("Open device %d", node_id);

    g_pKeyHandle = VCS_OpenDevice(pDeviceName, pProtocolStackName, pInterfaceName, pPortName, p_pErrorCode);

    if(g_pKeyHandle!=0 && *p_pErrorCode == 0)
    {

        unsigned int lBaudrate = 0;
        unsigned int lTimeout = 0;

        if(VCS_GetProtocolStackSettings(g_pKeyHandle, &lBaudrate, &lTimeout, p_pErrorCode)!=0)
        {
            if(VCS_SetProtocolStackSettings(g_pKeyHandle, g_baudrate, lTimeout, p_pErrorCode)!=0)
            {
                if(VCS_GetProtocolStackSettings(g_pKeyHandle, &lBaudrate, &lTimeout, p_pErrorCode)!=0)
                {
                    if(g_baudrate==(int)lBaudrate)
                    {
                        lResult = MMC_SUCCESS;
                    }
                }
            }
        }
    }
    else
    {
        g_pKeyHandle = 0;
    }

    delete []pDeviceName;
    delete []pProtocolStackName;
    delete []pInterfaceName;
    delete []pPortName;

        if(lResult == MMC_SUCCESS)
        {
            //LogInfo("Device openned correctly");
            ROS_INFO("Device openned correctly", node_id);
        }

    return lResult;
}

// Esta función debo de llamarla cuando cierre el programa.
// Está todavía pendiente de ser implementada
int CloseDevice(unsigned int* p_pErrorCode, int node_id)
{
        int lResult = MMC_FAILED;

        *p_pErrorCode = 0;

        LogInfo("Close device");

        if(VCS_CloseDevice(g_pKeyHandle, p_pErrorCode)!=0 && *p_pErrorCode == 0)
        {
                lResult = MMC_SUCCESS;
        }

        return lResult;
}

int PrepareDemo(unsigned int* p_pErrorCode)
{
    int lResult = MMC_SUCCESS;
    BOOL oIsFault = 0;
        for(int i = 1; i <= 6; i++)
        {
            g_usNodeId = i;

            if(VCS_GetFaultState(g_pKeyHandle, g_usNodeId, &oIsFault, p_pErrorCode ) == 0)
            {
                    LogError("VCS_GetFaultState", lResult, *p_pErrorCode);
                    lResult = MMC_FAILED;
            }

            if(lResult==0)
            {
                    if(oIsFault)
                    {
                            stringstream msg;
                            msg << "clear fault, node = '" << g_usNodeId << "'";
                            LogInfo(msg.str());

                            if(VCS_ClearFault(g_pKeyHandle, g_usNodeId, p_pErrorCode) == 0)
                            {
                                    LogError("VCS_ClearFault", lResult, *p_pErrorCode);
                                    lResult = MMC_FAILED;
                            }
                    }

                    if(lResult==0)
                    {
                            BOOL oIsEnabled = 0;

                            if(VCS_GetEnableState(g_pKeyHandle, g_usNodeId, &oIsEnabled, p_pErrorCode) == 0)
                            {
                                    LogError("VCS_GetEnableState", lResult, *p_pErrorCode);
                                    lResult = MMC_FAILED;
                            }

                            if(lResult==0)
                            {
                                    if(!oIsEnabled)
                                    {
                                            if(VCS_SetEnableState(g_pKeyHandle, g_usNodeId, p_pErrorCode) == 0)
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

void PrintHeader()
{
        SeparatorLine();

        LogInfo("Programa para eliminar fallos en la MCD Epos \nAutor: Jorge De Leon Rivas \nmail: jorge.deleon@upm.es");

        SeparatorLine();
}

int Demo(unsigned int* p_pErrorCode)
{
    int lResult = MMC_SUCCESS;
    unsigned int lErrorCode = 0;

        lResult = stand_up(g_pKeyHandle, g_usNodeId, lErrorCode);

        pause();

        //lResult = altern_tripod(g_pKeyHandle, g_usNodeId, lErrorCode);

        /*for(int i = 0; i <= 6; i++)
        {
            g_usNodeId = 6;

            lResult = DemoProfileVelocityMode(g_pKeyHandle, g_usNodeId, lErrorCode);

            if(lResult != MMC_SUCCESS)
            {
                    LogError("DemoProfileVelocityMode", lResult, lErrorCode);
            }
            else
            {
                    lResult = DemoProfilePositionMode(g_pKeyHandle, g_usNodeId, lErrorCode);

                    if(lResult != MMC_SUCCESS)
                    {
                            LogError("DemoProfilePositionMode", lResult, lErrorCode);
                    }
                    else
                    {
                            if(VCS_SetDisableState(g_pKeyHandle, g_usNodeId, &lErrorCode) == 0)
                            {
                                    LogError("VCS_SetDisableState", lResult, lErrorCode);
                                    lResult = MMC_FAILED;
                            }
                    }
            }
        }*/

    return lResult;
}



int stand_up(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode)
{
    int lResult = MMC_SUCCESS;

    const double tripod_one_position = 345; //330º
    const double tripod_two_position = 375;  // Esto debe ser 360º + 15º

    const double offset_inicial = 260;

    const double take_off_angle = 15;

    static double leg_actual_pos[6];
    static double leg_actual_error[6];
    static double leg_actual_error_old[6];
    //static double leg_desired_position[6] = {offset_inicial - fabs(motor_initial_offset_[0]), (offset_inicial + take_off_angle) - fabs(motor_initial_offset_[1]), (offset_inicial + take_off_angle) - fabs(motor_initial_offset_[2]), offset_inicial - fabs(motor_initial_offset_[3]), offset_inicial - fabs(motor_initial_offset_[4]), (offset_inicial + take_off_angle) - fabs(motor_initial_offset_[5])};
    //static double leg_desired_position[6] = {motor_initial_offset_[0], motor_initial_offset_[1], motor_initial_offset_[2] , motor_initial_offset_[3], motor_initial_offset_[4], motor_initial_offset_[5]};
    static double leg_desired_position[6] = {offset_inicial, (offset_inicial + take_off_angle), (offset_inicial + take_off_angle), offset_inicial, offset_inicial, (offset_inicial + take_off_angle)};

    const int tripod_one[3] = {1, 4, 5};
    const int tripod_two[3] = {2, 3, 6};

    static bool pose = false;

    static bool leg_reached_offset[6] = {false, false, false, false, false, false};

    static int safe_counter[6] = {0, 0, 0, 0, 0, 0};


    // Activar el modo de velocidad

    for(int i = 1; i <= 6; i++)
    {
        p_usNodeId = i;
        if(VCS_ActivateProfileVelocityMode(p_DeviceHandle, p_usNodeId, &p_rlErrorCode) == 0)
        {
            LogError("VCS_ActivateProfileVelocityMode", lResult, p_rlErrorCode);
            lResult = MMC_FAILED;
        }

        std::cout << "Position desired motor " << i << " : " << leg_desired_position[i-1] << std::endl;
    }


    // Movimiento de las patas al offset
    while(!pose)
    {

        for(int i = 1; i <= 6; i++)
        {
            //unsigned int ulErrorCode = 0;
            //joint_state_publish(g_pKeyHandle, g_usNodeId, ulErrorCode);

            if(!leg_reached_offset[i-1])
            {

                int leg_actual_pos_raw;
                int p_usNodeId = i;
                VCS_GetPositionIs(p_DeviceHandle, p_usNodeId, &leg_actual_pos_raw, &p_rlErrorCode);
                leg_actual_pos[i-1] = fabs((leg_actual_pos_raw / 2000.0) * 2 * M_PI);
                leg_actual_error[i-1] = fabs(leg_desired_position[i-1] - leg_actual_pos[i-1]);

                // Muestra el error de posicion si la pata tiene que moverse
                if((leg_actual_error_old[i-1] != leg_actual_error[i-1]))
                {
                    stringstream msg;
                    msg << "leg = " << p_usNodeId << ", position = " << leg_actual_pos[i-1] << ", error = " << leg_actual_error[i-1] << endl;
                    LogInfo(msg.str());
                }

                // Si el error de consigna es menor a 3 grados se da como buena la posicion

                if((leg_actual_error[i-1] <= 4))
                {
                    leg_reached_offset[i-1] = true;
                    VCS_HaltVelocityMovement(p_DeviceHandle, p_usNodeId, &p_rlErrorCode);
                    safe_counter[i-1] = 0;
                    ROS_INFO("Motor %i en position", i);
                }

                if(!leg_reached_offset[i-1])
                {
                    int velocity_tripod_1 = 100;
                    int velocity_tripod_2 = 125;
                    int velocity;

                    static bool flag = true;
                    if(flag)
                    {
                        ROS_INFO("tripod one: %d -- tripod two: %d", velocity_tripod_1, velocity_tripod_2);
                        //n = 2;
                        flag = !flag;
                    }

                    if(safe_counter[i-1] > 1000)
                    {
                        velocity_tripod_1 = 2 * velocity_tripod_1;
                        velocity_tripod_2 = 2 * velocity_tripod_2;
                    }

                    if((i == 2) || (i == 4) || (i == 6))
                    {
                        velocity = velocity_tripod_2;
                    }

                    else
                        velocity = -velocity_tripod_1;

                    if(VCS_MoveWithVelocity(p_DeviceHandle, p_usNodeId, velocity, &p_rlErrorCode) == 0)
                    {
                        lResult = MMC_FAILED;
                        LogError("VCS_MoveWithVelocity", lResult, p_rlErrorCode);
                        break;
                    }

                    else
                        safe_counter[i-1]++;
                }

                leg_actual_error_old[i-1] = leg_actual_error[i-1];
            }
        }

        if(leg_reached_offset[0] && leg_reached_offset[1] && leg_reached_offset[2] && leg_reached_offset[3] && leg_reached_offset[4] && leg_reached_offset[5])
        {
            pose = true;
            stringstream msg;
            msg << "Pose = true" << endl;
            LogInfo(msg.str());
            return lResult;
        }
    }

    //pause();
}


void joint_state_publish(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode)
{
    sensor_msgs::JointState joint_msg;

    string motor_names[6] = {"motor_1", "motor_2", "motor_3", "motor_4", "motor_5", "motor_6"};
    double leg_actual_pos[6];
    double leg_actual_vel[6];
    double leg_actual_current[6];
    double leg_actual_effort[6];

    for(int i = 0; i <= 5; i++)
    {
        //ROS_INFO("Entro");
        p_usNodeId = i+1;
        int leg_actual_pos_raw;
        VCS_GetPositionIs(p_DeviceHandle, p_usNodeId, &leg_actual_pos_raw, &p_rlErrorCode);
        leg_actual_pos[i] = fabs((leg_actual_pos_raw / 2000.0) * 2 * M_PI);

        int leg_actual_vel_raw;
        VCS_GetVelocityIs(p_DeviceHandle, p_usNodeId, &leg_actual_vel_raw, &p_rlErrorCode);
        leg_actual_vel[i] = leg_actual_vel_raw;

        short leg_actual_current_raw;
        VCS_GetCurrentIs(p_DeviceHandle, p_usNodeId, &leg_actual_current_raw, &p_rlErrorCode);
        leg_actual_current[i] = leg_actual_current_raw  / 1000.0; //mA -> A
        double torque_constant = 2.6;
        leg_actual_effort[i] = leg_actual_current[i] * torque_constant;

        joint_msg.name.push_back(motor_names[i]);
        joint_msg.position.push_back(leg_actual_pos[i]);
        joint_msg.velocity.push_back(leg_actual_vel[i]);
        joint_msg.effort.push_back(leg_actual_effort[i]);

        //ROS_INFO("motor: %d -- pos: %.2f -- vel: %.2f -- torque: %.2f", p_usNodeId, leg_actual_pos[i], leg_actual_vel[i], leg_actual_effort[i]);

        if(i == 5)
        {
            joint_states_pub.publish(joint_msg);

        }
    }

    //joint_msg.name[0]
}


/*
int prepare_legs(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode)
{
    string motor_names[6] = {"motor_1", "motor_2", "motor_3", "motor_4", "motor_5", "motor_6"};
    double leg_actual_current[6];
    double leg_actual_effort[6];

    static bool motor_in_position[6] = {false, false, false, false, false, false};

    int lResult = MMC_SUCCESS;

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

    for(int i = 0; i <= 5; i++)
    {
        while(!motor_in_position[i])
        {
            p_usNodeId = i+1;

            short leg_actual_current_raw;
            VCS_GetCurrentIs(p_DeviceHandle, p_usNodeId, &leg_actual_current_raw, &p_rlErrorCode);
            leg_actual_current[i] = leg_actual_current_raw / 1000.0; //mA -> A
            double torque_constant = 2.6;
            leg_actual_effort[i] = leg_actual_current[i] * torque_constant;

            int velocity = -250;

            if((i == 1) || (i == 3) || (i == 5))
            {
                velocity = +250;
            }

            else
                velocity = velocity;

            if(fabs(leg_actual_effort[i]) < 3)
            {
                if(VCS_MoveWithVelocity(p_DeviceHandle, p_usNodeId, velocity, &p_rlErrorCode) == 0)
                {
                    lResult = MMC_FAILED;
                    LogError("VCS_MoveWithVelocity", lResult, p_rlErrorCode);
                    break;
                }
            }

            //ROS_INFO("Motor: %i -- Corriente: %.2f [mA] -- Torque: %.2f", p_usNodeId, leg_actual_current[i], fabs(leg_actual_effort[i]));

            if(fabs(leg_actual_effort[i]) >= 3)
            {
                VCS_HaltVelocityMovement(p_DeviceHandle, p_usNodeId, &p_rlErrorCode);

                int leg_actual_pos_raw;
                static double leg_actual_pos[6];
                VCS_GetPositionIs(p_DeviceHandle, p_usNodeId, &leg_actual_pos_raw, &p_rlErrorCode);
                leg_actual_pos[i] = ((leg_actual_pos_raw / 4000.0) * 2 * M_PI);
                ROS_INFO("Motor %i parado -- position: %.2f", p_usNodeId, leg_actual_pos[i]);
                motor_in_position[i] = true;
                motor_initial_offset_[i] = leg_actual_pos[i];
            }
        }   //while(!motor_in_position[i])


    }   //for(int i = 0; i <= 5; i++)

    if(motor_in_position[0] && motor_in_position[1] && motor_in_position[2] && motor_in_position[3] && motor_in_position[4] && motor_in_position[5])
    {
        return lResult = MMC_SUCCESS;
    }

}
*/

/*
int altern_tripod(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode)
{
    ROS_INFO("Tripode alterno");

    int lResult = MMC_SUCCESS;

    const int tripod_one[3] = {1, 4, 5};
    static bool pose = false;

    static bool leg_reached_offset[3] = {false, false, false};
    static double leg_desired_position[3] = {290, 290, 290};


    static double leg_actual_pos[6];
    static double leg_actual_error[6];
    static double leg_actual_error_old[6];

    for(int i = 1; i <= 3; i++)
    {
        p_usNodeId = tripod_one[i-1];

        if(!leg_reached_offset[i-1])
        {

            int leg_actual_pos_raw;
            VCS_GetPositionIs(p_DeviceHandle, p_usNodeId, &leg_actual_pos_raw, &p_rlErrorCode);
            leg_actual_pos[i-1] = fabs((leg_actual_pos_raw / 2000.0) * 2 * M_PI);
            leg_actual_error[i-1] = fabs(leg_desired_position[i-1] - leg_actual_pos[i-1]);

            if((leg_actual_error_old[i-1] != leg_actual_error[i-1]))
            {
                stringstream msg;
                msg << "leg = " << p_usNodeId << ", position = " << leg_actual_pos[i-1] << ", error = " << leg_actual_error[i-1] << endl;
                LogInfo(msg.str());
            }

            if((leg_actual_error[i-1] <= 4))
            {
                leg_reached_offset[i-1] = true;
                VCS_HaltVelocityMovement(p_DeviceHandle, p_usNodeId, &p_rlErrorCode);
                //safe_counter[i-1] = 0;
                ROS_INFO("Motor %i en position", i);
            }

            double velocity_tripod_1;
            double velocity;

            if(!leg_reached_offset[i-1])
                velocity_tripod_1 = 100;

            if(i == 2)
                velocity = -velocity_tripod_1;

            else
                velocity = velocity_tripod_1;

            if(VCS_MoveWithVelocity(p_DeviceHandle, p_usNodeId, velocity, &p_rlErrorCode) == 0)
            {
                lResult = MMC_FAILED;
                LogError("VCS_MoveWithVelocity", lResult, p_rlErrorCode);
                break;
            }

            leg_actual_error_old[i-1] = leg_actual_error[i-1];
        }

        if(leg_reached_offset[0] && leg_reached_offset[1] && leg_reached_offset[2])
        {
            pose = true;
            stringstream msg;
            msg << "Pose = true" << endl;
            LogInfo(msg.str());
            return lResult;
        }
    }

}
*/

int main(int argc, char** argv) {

    //ROS_INFO("Tus muertos");

    //creating the nodde
    ros::init(argc, argv, "alt_sensor_node");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();

    //creating a publisher
    //ros::Publisher value_pub=nh.advertise<altitude_sensor::sensor_data>("altitude", 5);

    //Publisher for joints_states
    //ros::Publisher joint_states_pub = nh.advertise<sensor_msgs::JointState>("hexapodo_joint_states", 1000);
    joint_states_pub = nh.advertise<sensor_msgs::JointState>("hexapodo_joint_states", 1000);

    ros::Rate loop_rate(sensor_frequency);

    cereal::CerealPort device;
    char reply[REPLY_SIZE];

    altitude_sensor::sensor_data message;


    //setting default device path for the sensor
    nh.param("serial_port", serial_port, std::string("/dev/ttyS1"));


    //reading the string from serial port
    //try{ device.open(serial_port.c_str() , 115200); }
    try{ device.open("/dev/ttyS1" , 115200); }
        catch(cereal::Exception& e)
        {
            ROS_FATAL("Failed to open the serial port.");
            ROS_BREAK();
        }
        ROS_INFO("The serial port is opened.");

        int lResult = MMC_FAILED;
        unsigned int ulErrorCode = 0;

        //SetDefaultParameters();

        //PrintSettings();

        /**** CLEAR ERROS && OPEN DEVICES *****/
        for (int i = 1; i < 7; i++)
        {
            int node_id = i;

            SetDefaultParameters(node_id);

            if((lResult = OpenDevice(&ulErrorCode, node_id))!=MMC_SUCCESS)
            {
                    LogError("OpenDevice", lResult, ulErrorCode);
                    return lResult;
            }
        }

        if((lResult = PrepareDemo(&ulErrorCode))!=MMC_SUCCESS)
        {
                LogError("PrepareDemo", lResult, ulErrorCode);
                return lResult;
        }


        //std::cout << "Todo bien" << std::endl;

        /*if((lResult = Demo(&ulErrorCode))!=MMC_SUCCESS)
        {
            LogError("Demo", lResult, ulErrorCode);
            return lResult;
        }*/

        /*if((lResult = prepare_legs(g_pKeyHandle, g_usNodeId, ulErrorCode))!=MMC_SUCCESS)
        {
            LogError("Prepare legs", lResult, ulErrorCode);
            return lResult;
        }*/

        /*for(int i = 0; i < 6; i++)
        {
            std::cout << "Offset del motor " << i+1 << ": " << motor_initial_offset_[i] << " grados" <<std::endl;
        }

        if((lResult = stand_up(g_pKeyHandle, g_usNodeId, ulErrorCode))!=MMC_SUCCESS)
        {
            LogError("Prepare legs", lResult, ulErrorCode);
            return lResult;
        }

        if((lResult = altern_tripod(g_pKeyHandle, g_usNodeId, ulErrorCode))!=MMC_SUCCESS)
        {
            LogError("Prepare legs", lResult, ulErrorCode);
            return lResult;
        }*/

        //converting string into float
    while(ros::ok())
    {
        ROS_INFO("While");

        for(int i = 0; i < 1; i++)
        {
            if((lResult = Demo(&ulErrorCode))!=MMC_SUCCESS)
            {
                LogError("Demo", lResult, ulErrorCode);
                return lResult;
            }
            i++;
        }

        joint_state_publish(g_pKeyHandle, g_usNodeId, ulErrorCode);
        //loop_rate.sleep();

        //ros::spinOnce();
    }

    for (int i = 1; i < 7; i++)
    {
        int node_id = i;

        //SetDefaultParameters(node_id);

        if((lResult = CloseDevice(&ulErrorCode, node_id))!=MMC_SUCCESS)
        {
                LogError("CloseDevice", lResult, ulErrorCode);
                return lResult;
        }
    }
    return 0;
    device.close();
}
