#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <movement_orders/movement_orders.h>

int move_order = 0;

void cmd_vel_cb(const geometry_msgs::Twist& msg)
{
    /************
     * move_order->variable para indicar el movimiento del robot
     * mover_order =  1 -> Forward
     * mover_order = -2 -> Left
     * mover_order =  2 -> Right
     * mover_order =  0 -> Stop
     * *********/

    if((msg.linear.x > 0) && (msg.angular.z == 0))
        move_order =  1;

    else if((msg.linear.x == 0) && (msg.angular.z < 0))
        move_order = -2;

    else if((msg.linear.x == 0) && (msg.angular.z > 0))
        move_order =  2;

    else {
        move_order =  0;
    }

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_movement");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("cmd_vel", 1, cmd_vel_cb);

    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();

    movement_orders clhero_orders;

    while(ros::ok())
    {
        switch(move_order){
            case 1: //foward();
                    break;

            case 2: //right();
                    break;

            case -2: //left();
                     break;

            default: clhero_orders.stop(1);
                     break;
        }
        //break;

    } // while(ros::ok())


    ros::spin();

    return 0;
}
