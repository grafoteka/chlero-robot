/******************************************
 * No es posible correr este programa en paralelo con otro que haya abierto el
 * mismo puerto serial.
 * No se pueden realizar por un mismo puerto 2 conexiones simultáneas.
 * Hay que cerrar la activa para poder realizar la segunda.
 */
//============================================================================
// Name        : motores_estado_lectura.cpp
// Author      : Jorge De Leon
// Version     :
// Date	       : 04-FEBRERO-2019
// Description : Program to read the state of the motores (position, velocity, torque)
//               This program, also, tries to verify if is possible to open a second
//               serial comunication with the same device.
//============================================================================

// Librerias esenciales
#include <ros/ros.h>
#include <cereal_port/CerealPort.h>
#include <std_msgs/String.h>

#define REPLY_SIZE 20
#define TIMEOUT 1000

const unsigned int sleep_rate = 100; /*sleep rate*/

std::string serial_port;

typedef void* HANDLE;
typedef int BOOL;

//using namespace std;

#ifndef MMC_SUCCESS
    #define MMC_SUCCESS 0
#endif

#ifndef MMC_FAILED
    #define MMC_FAILED 1
#endif

#ifndef MMC_MAX_LOG_MSG_SIZE
    #define MMC_MAX_LOG_MSG_SIZE 512
#endif

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motores_estado_lectura");
    ros::NodeHandle nh;

    ros::Rate loop_rate(sleep_rate);

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
}
