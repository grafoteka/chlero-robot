#include <ros/ros.h>
#include <std_msgs/String.h>
#include <cereal_port/CerealPort.h>
#include <hexapod_class/hexapod_class.h>
#include <sensor_msgs/JointState.h>

#define REPLY_SIZE 20
#define TIMEOUT 1000

const unsigned int sensor_frequency = 100; /*sensor frequency in hz*/

std::string serial_port;


typedef void* HANDLE;
typedef int BOOL;

using namespace std;


//    EAppMode g_eAppMode = AM_DEMO;


#ifndef MMC_SUCCESS
    #define MMC_SUCCESS 0
#endif

#ifndef MMC_FAILED
    #define MMC_FAILED 1
#endif

#ifndef MMC_MAX_LOG_MSG_SIZE
    #define MMC_MAX_LOG_MSG_SIZE 512
#endif



/*int Demo(unsigned int* p_pErrorCode)
{
    int lResult = MMC_SUCCESS;
    unsigned int lErrorCode = 0;

        //lResult = hexapod_robot_class.stand_up(g_pKeyHandle, g_usNodeId, lErrorCode);

        //lResult = stand_up(g_pKeyHandle, g_usNodeId, lErrorCode);

        pause();

        //lResult = altern_tripod(g_pKeyHandle, g_usNodeId, lErrorCode);



    return lResult;
}*/

/*
bool stand_up(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode)
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
                stringstream msg;
                msg << "leg = " << p_usNodeId << ", position = " << leg_actual_pos[i-1] << ", error = " << leg_actual_error[i-1] << endl;
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
            stringstream msg;
            msg << "Pose = true" << endl;
            LogInfo(msg.str());
        }
    }

    return true;

    //pause();
}*/

/*void joint_state_publish(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode)
{
    sensor_msgs::JointState joint_msg;

    string motor_names[6] = {"motor_1", "motor_2", "motor_3", "motor_4", "motor_5", "motor_6"};
    double leg_actual_pos[6];
    double leg_actual_vel[6];
    double leg_actual_current[6];
    double leg_actual_effort[6];

    for(int i = 0; i <= 5; i++)
    {
        ROS_INFO("Entro");
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

    }
    //joint_msg.name[0]
}*/


int main(int argc, char** argv) {

    //creating the nodde
    ros::init(argc, argv, "hexapod_robot");
    ros::NodeHandle nh;

    //creating a publisher
    //ros::Publisher value_pub=nh.advertise<altitude_sensor::sensor_data>("altitude", 5);

    //Publisher for joints_states
    //ros::Publisher joint_states_pub = nh.advertise<sensor_msgs::JointState>("hexapodo_joint_states", 1);

    ros::Rate loop_rate(sensor_frequency);

    cereal::CerealPort device;
    char reply[REPLY_SIZE];


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

    hexapod_class hexapod_robot_class;

    if((lResult = hexapod_robot_class.epos_setup())!=MMC_SUCCESS)
    {
            return lResult;
    }




    while(ros::ok()) {
        ROS_INFO("While");

        //joint_state_publish(g_pKeyHandle, g_usNodeId, ulErrorCode);
        loop_rate.sleep();

        ros::spinOnce();
    }
    return 0;
}
