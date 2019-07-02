//=====================================================================
//	Author:	Raúl Cebolla Arroyo
//	File:
//	Version:
//	Description:
//	Changelog:
//=====================================================================

//----------------------------------------------------
//    Includes
//----------------------------------------------------

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <clhero_gait_controller/LegCommand.h>
#include <clhero_gait_controller/LegState.h>
#include <iostream>
#include <vector>
#include <string>
#include <mutex>
#include <cmath>
#include <thread>
#include <epos_functions/epos_functions.h>

//----------------------------------------------------
//    Defines
//----------------------------------------------------

#define LOOP_RATE 500 //Rate at which the node checks for callbacks
#define LEG_NUMBER 6 //Number of legs of the robot
#define CONTROL_RATE 100 //Rate which the node sends the control of each leg
#define PI 3.14159265359
#define ANG_THR 0.03490658503988659 //2*(2*PI)/360
#define ANG_V 0.17453292519943295 //10*(2*PI)/360
#define STATE_UPDATE_RATE 200 //Rate at which the state of the legs is update [Hz]
#define POS_COMMAND_THR 0.08726646259971647 //Threshold in which a position command is considered the same [0.5 deg]
#define DEFAULT_VEL (2*3.141592653589793) // 0.5 rev/s = 30 rpm
#define DEFAULT_ACCEL 1256.6370614359172 // 1200 rpm/s
#define DEFAULT_DECEL 1256.6370614359172 // 1200 rpm/s
#define ERROR_THR 1e-6

//----------------------------------------------------
//    Class definitions
//----------------------------------------------------

class CommandMsgManager{

	clhero_gait_controller::LegCommand current_command;
	clhero_gait_controller::LegCommand fixed_command;
	epos_functions* epos;
	std::vector<bool> is_new_command;

	//Function that checks if one leg has its turning direction with the opposite sign
	bool checkNegativeMotor (int motor);

	void fixNewCommand ();

public:

	CommandMsgManager(epos_functions* e);

	void evaluateNewCommand(const clhero_gait_controller::LegCommand::ConstPtr& msg);

	void commandAllMotors();

};

//----------------------------------------------------
//    Global variables
//----------------------------------------------------

//Publisher of the legs_state_msg
ros::Publisher legs_state_pub;

//Publisher of each of the command msgs
std::vector<ros::Publisher> controller_command_pub;

//EPOS control functions class instantiation
epos_functions* epos_f;

//Command msg manager
CommandMsgManager* com_man;

//----------------------------------------------------
//    Functions
//----------------------------------------------------

//Function that maps the motors
int mapMotor (int motor){
	switch (motor){
		case 1:
			return 6;
		case 2:
			return 5;
		case 3:
			return 4;
		case 4:
			return 3;
		case 5:
			return 2;
		case 6:
			return 1;
		default:
			return 0;
	}
}

//Function that turns rad/s into rpm
inline double rads2rpm (double rads){
	return rads*60/(2*PI);
}

//Function that checks if one leg has its turning direction with the opposite sign
inline bool checkNegativeMotor (int motor){
	switch (motor){
		case 1:
			return true;
		case 3:
			return true;
		case 5:
			return true;
		default:
			return false;
	}
	return false;
}

//Function that turns a relative position command [0, 2pi) into an absolute position based
//on the accumulate position of the motor
double turnAbsolutePosition (double pos_command, double vel_command, double curr_position){
	
	long n = 0;
	double fixed_command;

	//Gets the number of turns
	n = trunc(curr_position/(2*PI));

	//Depending on the movement's direction, the final position shall be corrected 
	if(vel_command > 0){
		fixed_command = 2*PI*n + pos_command;		
		//If the final position is lower than the current position on forward movement
		if((fixed_command < curr_position) && (fabs(fixed_command - curr_position) > POS_COMMAND_THR)){
			fixed_command += 2*PI;
		}
	}else{
		fixed_command = 2*PI*n - (2*PI - pos_command);		
		//If the final position is greater than the current position on backward movement
		if((fixed_command > curr_position) && (fabs(fixed_command - curr_position) > POS_COMMAND_THR)){
			fixed_command -= 2*PI;
		}
	}

	return fixed_command;
}

//Function that turns the readings of position into a range of [0, 360]
double fixAngle (double angle){
  return (angle - 2*PI*trunc(angle/(2*PI)));
}

//Callback for leg command msgs
void legCommandCallback (const clhero_gait_controller::LegCommand::ConstPtr& msg){

	com_man->evaluateNewCommand(msg);
	com_man->commandAllMotors();

 	return;
}

//Thread that periodically updates the state of the legs
void StateUpdateThread (){

	//Leg State msg
	clhero_gait_controller::LegState leg_state_msg;

	//Creates the ros rate
	ros::Rate state_update_rate (STATE_UPDATE_RATE);

	//Core loop of the thread
	while(ros::ok()){

		double position, velocity;
		int n = 0;

		//For each leg
		for(int i = 0; i < LEG_NUMBER; i++){
			
			position = epos_f->GetPosition(mapMotor(i+1));
			velocity = epos_f->GetVelocity(mapMotor(i+1));
			n = trunc(position/(2*PI));
			position -= 2*PI*n;
			
			if (position < 0){
				position += 2*PI;
			}

			if(checkNegativeMotor(mapMotor(i+1))){
				position = 2*PI - position;
				velocity = (-1.0)*velocity;
			}
			
			leg_state_msg.pos.push_back(position);
    		leg_state_msg.vel.push_back(velocity);
    		leg_state_msg.torq.push_back(0);
		}

		//Publishes the msg
		legs_state_pub.publish(leg_state_msg);

		leg_state_msg.pos.clear();
		leg_state_msg.vel.clear();
		leg_state_msg.torq.clear();

		//Sleeps for each loop
		state_update_rate.sleep();
	}

	return;
}

//----------------------------------------------------
//    Class definitions
//----------------------------------------------------


//Function that checks if one leg has its turning direction with the opposite sign
bool CommandMsgManager::checkNegativeMotor (int motor){
	switch (motor){
		case 1:
			return true;
		case 3:
			return true;
		case 5:
			return true;
		default:
			return false;
	}
	return false;
}

void CommandMsgManager::fixNewCommand (){
	//For each leg
	for(int i=0; i<LEG_NUMBER; i++){
		//If there is a new command to be treated
		if(is_new_command[i]){
			//Command for this leg
			fixed_command.pos[i] = current_command.pos[i];
			fixed_command.vel[i] = current_command. vel[i];
			fixed_command.acel[i] = current_command.acel[i];
			fixed_command.decel[i] = current_command.decel[i];	
			fixed_command.new_acel_profile[i].data = current_command.new_acel_profile[i].data;
			fixed_command.position_command[i].data = current_command.position_command[i].data;

			//First checks the mode: position or velocity
			if(current_command.position_command[i].data){
				//Position command

				//Absolute position
				double abs_position;

				//Activates the position mode
				this->epos->ActivateProfilePosition(mapMotor(i+1));
				//Corrects the command if the motor is negative-turn
				if(this->checkNegativeMotor(mapMotor(i+1))){
					fixed_command.pos[i] = 2*PI - fixed_command.pos[i];
					fixed_command.vel[i] = (-1.0)*fixed_command.vel[i]; 
				}
				//Checks if a new profile shall be set
				if(current_command.new_acel_profile[i].data){
					//If so, uploads the acceleration and decceleration
					epos->SetPositionProfile(mapMotor(i+1), fabs(fixed_command.vel[i]), fixed_command.acel[i], fixed_command.decel[i]);
				}

				//Gets the absolute position based on the position command, velocity and current position
				fixed_command.pos[i] = turnAbsolutePosition(fixed_command.pos[i], fixed_command.vel[i], this->epos->GetPosition(mapMotor(i+1)));
				//Once the position has been set sends the order
				//epos->MoveToPosition(mapMotor(i+1), abs_position, true, true);
			}else{
				//Velocity command

				//Activates the velocity profile
				this->epos->ActivateProfileVelocity(mapMotor(i+1));
				//Corrects the command if the motor is negative-turn
				if(this->checkNegativeMotor(mapMotor(i+1))){
					fixed_command.pos[i] = 2*PI - fixed_command.pos[i];
					fixed_command.vel[i] = (-1.0)*fixed_command.vel[i];
				}
				//Checks if a new profile shall be set
				if(current_command.new_acel_profile[i].data){
					//If so, uploads the acceleration and decceleration
					this->epos->SetVelocityProfile(mapMotor(i+1), fixed_command.acel[i], fixed_command.decel[i]);
				}
				//Once the position has been set sends the order
				//epos->MoveWithVelocity(mapMotor(i+1), current_command.vel[i]);
			}
		}
	}
}

CommandMsgManager::CommandMsgManager(epos_functions* e){
	
	this->epos = e;

	for(int i=0; i<LEG_NUMBER; i++){

		is_new_command.push_back(false);
		
		current_command.pos[i] = -1003;
		current_command.vel[i] = current_command.acel[i] = current_command.decel[i] = -10e6;
		current_command.new_acel_profile[i].data = false;
		current_command.position_command[i].data = false;

		fixed_command.pos[i] = -1003;
		fixed_command.vel[i] = fixed_command.acel[i] = fixed_command.decel[i] = -10e6;
		fixed_command.new_acel_profile[i].data = false;
		fixed_command.position_command[i].data = false;
	}

}

void CommandMsgManager::evaluateNewCommand(const clhero_gait_controller::LegCommand::ConstPtr& msg){
	
	for(int i=0; i<LEG_NUMBER; i++){

		if(fabs(current_command.pos[i] - msg->pos[i]) > ERROR_THR){
			
			is_new_command[i] = true;
			continue;

		}else if(fabs(current_command.vel[i] - msg->vel[i]) > ERROR_THR){

			is_new_command[i] = true;
			continue;

		}else if(fabs(current_command.acel[i] - msg->acel[i]) > ERROR_THR){

			is_new_command[i] = true;
			continue;

		}else if(fabs(current_command.decel[i] - msg->decel[i]) > ERROR_THR){

			is_new_command[i] = true;
			continue;

		}else if(current_command.new_acel_profile[i].data != msg->new_acel_profile[i].data){
			
			is_new_command[i] = true;
			continue;

		}else if(current_command.position_command[i].data != msg->position_command[i].data){
			
			is_new_command[i] = true;
			continue;
							
		}
	}

	for(int i=0; i<LEG_NUMBER; i++){
		if(is_new_command[i]){
			current_command.pos[i] = msg->pos[i];
			current_command.vel[i] = msg->vel[i];
			current_command.acel[i] = msg->acel[i];
			current_command.decel[i] = msg->decel[i];
			current_command.new_acel_profile[i].data = msg->new_acel_profile[i].data;
			current_command.position_command[i].data = msg->position_command[i].data;			
		}
	}

	this->fixNewCommand();
	return;
}

void CommandMsgManager::commandAllMotors(){
	for(int i=0; i<LEG_NUMBER; i++){
		if(is_new_command[i]){
			if(fixed_command.position_command[i].data){
				//Position command
				this->epos->MoveToPosition(mapMotor(i+1), fixed_command.pos[i], true, true);
			}else{
				//Velocity command
				this->epos->MoveWithVelocity(mapMotor(i+1), fixed_command.vel[i]);
			}
		}
		is_new_command[i] = false;
	}
	return;
}

//----------------------------------------------------
//    Main function
//----------------------------------------------------

int main(int argc, char **argv){

  //----------------------------------------------------
  //    ROS starting statements
  //----------------------------------------------------

  ros::init(argc, argv, "clhero_simulation_interface");
  ros::NodeHandle nh;

  ros::Rate loop_rate (LOOP_RATE);

  //Creates the maxon motors'handler
  epos_f = new epos_functions();

  //Sets the default profile
  for(int i = 0; i < LEG_NUMBER; i++){
  	epos_f->ActivateProfilePosition(mapMotor(i+1));
  	epos_f->SetPositionProfile(mapMotor(i+1), DEFAULT_VEL, DEFAULT_ACCEL, DEFAULT_DECEL);
  	//epos_f->SetVelocityProfile(mapMotor(i+1), DEFAULT_ACCEL, DEFAULT_DECEL);
  }

  //Creates the msg manager
  com_man = new CommandMsgManager(epos_f);

  //Publishers
  legs_state_pub = nh.advertise<clhero_gait_controller::LegState>("legs_state", 1);
  
  //Topics subscription
  ros::Subscriber leg_command_sub = nh.subscribe("legs_command", 1, legCommandCallback);
  //ros::Subscriber joint_states_sub = nh.subscribe("/hexapodo/joint_states", 1000, jointStatesCallback);
  
  //threads which helds the status publishing function
  std::thread state_update_thr (StateUpdateThread);

  //----------------------------------------------------
  //    Core loop of the node
  //----------------------------------------------------
  while(ros::ok()){
  	ros::spinOnce();
  	loop_rate.sleep();	
  }

  //----------------------------------------------------
  //    End of node statements
  //----------------------------------------------------

  state_update_thr.join();

  delete epos_f;
  delete com_man;

  return 0;

}
