
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
#include <clhero_gait_controller/PatternCommand.h>
#include <iostream>
#include <vector>
#include <string>
#include <cstring>
#include <mutex>
#include <cmath>
#include <thread>
#include <epos_functions/epos_functions.h>


//----------------------------------------------------
//    Defines
//----------------------------------------------------

#define LOOP_RATE 1000 //Rate at which the node checks for callbacks
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

/*class CommandMsgManager{

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

};*/

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
//CommandMsgManager* com_man;

int pattern_mode_number  = 0;
int pattern_state_number = 0;

//----------------------------------------------------
//    Functions
//----------------------------------------------------

//Function that maps the motors
/*int mapMotor (int motor){
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
}*/

//Function that turns rad/s into rpm
/*inline double rads2rpm (double rads){
	return rads*60/(2*PI);
}*/

//Function that checks if one leg has its turning direction with the opposite sign
/*inline bool checkNegativeMotor (int motor){
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
}*/

//Function that turns a relative position command [0, 2pi) into an absolute position based
//on the accumulate position of the motor
/*double turnAbsolutePosition (double pos_command, double vel_command, double curr_position){
	
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
}*/
/*
//Function that turns the readings of position into a range of [0, 360]
double fixAngle (double angle){
  return (angle - 2*PI*trunc(angle/(2*PI)));
}*/

//Callback for leg command msgs
void legCommandCallback (const clhero_gait_controller::LegCommand::ConstPtr& msg){

	/*float current_pos = msg->pos[0];
	float current_vel = msg->vel[0];
	float current_acel = msg->acel[0];
	float current_decel = msg->decel[0];
	bool current_new_acel_profile = msg->new_acel_profile[0].data;
	bool current_position_command = msg->position_command[0].data;

	ROS_INFO("pata 1 : pos = %.2f -- vel = %.2f -- acel = %.2f -- decel = %.2f", current_pos, current_vel, current_acel, current_decel);
	
	epos_f->SetPositionProfile(1, current_vel, current_acel, current_decel);
	epos_f->MoveToPosition(1, current_pos, true, true);*/

	//com_man->evaluateNewCommand(msg);
	//com_man->commandAllMotors();

 	return;
}

void patternCommandCallback (const clhero_gait_controller::PatternCommand::ConstPtr& msg){
	
	// Modo de marcha
	std::string gait_mode = msg->pattern_name;
	
	// Estado de la marcha (start/stop)
	std::string gait_state = msg->order;

	// Modo de marcha filtrado
	static std::string gait_mode_selected = "";
	//static std::string gait_old = "";

	// Estado de la marcha
	static std::string gait_state_selected = "";

	//Comparo el modo de marcha seleccionado
	//Si es igual, significa que estoy el mismo modo. No hago nada
	if(gait_mode_selected.compare(gait_mode) != 0)
	{
		//ROS_INFO("Nuevo modo");
		gait_mode_selected.clear();
		gait_mode_selected = gait_mode_selected + gait_mode;
		//ROS_INFO_STREAM(gait_mode_selected);
	}

	if(gait_state_selected.compare(gait_state) != 0)
	{
		//ROS_INFO("Nuevo estado");
		gait_state_selected.clear();
		gait_state_selected = gait_state + gait_state_selected;
		//ROS_INFO_STREAM(gait_state_selected);
	}

	/*if(gait_actual.compare(gait_mode) != 0)
	{
		ROS_INFO("Nuevo modo");
		gait_actual = gait_actual + gait_mode;
		ROS_INFO_STREAM(gait_actual);
	}*/

	if(gait_mode_selected.compare("alternating_tripod") == 0)
	{
		pattern_mode_number = 1;
	} else if (gait_mode_selected.compare("turn_right_tripod") == 0)
	{
		pattern_mode_number = 2;

	} else if (gait_mode_selected.compare("turn_left_tripod") == 0)
	{
		pattern_mode_number = -2;
	} else if (gait_mode_selected.compare("stand_up") == 0)
	{
		pattern_mode_number = 10;
	} else if (gait_mode_selected.compare("get_down") == 0)
	{
		pattern_mode_number = -10;
	} else
		pattern_mode_number = 0;

	// Start / Stop
	if(gait_state_selected.compare("start") == 0)
	{
		pattern_state_number = 1;
	}
	else
		pattern_state_number = 0;

	//ROS_INFO("Pattern number = %d -- Pattern state = %d", pattern_mode_number, pattern_state_number);


}

//Thread that periodically updates the state of the legs
/*void StateUpdateThread (){

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
}*/

//----------------------------------------------------
//    Class definitions
//----------------------------------------------------


//Function that checks if one leg has its turning direction with the opposite sign
/*bool CommandMsgManager::checkNegativeMotor (int motor){
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
}*/

/*void CommandMsgManager::fixNewCommand (){
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
}*/

/*CommandMsgManager::CommandMsgManager(epos_functions* e){
	
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

}*/

/*void CommandMsgManager::evaluateNewCommand(const clhero_gait_controller::LegCommand::ConstPtr& msg){
	
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
			
			ROS_INFO("pata %d : pos = %.2f -- vel = %.2f -- acel = %.2f -- decel = %.2f", i, current_command.pos[i], current_command.vel[i], current_command.acel[i], current_command.decel[i]);
		}
	}

	this->fixNewCommand();
	return;
}*/

/*void CommandMsgManager::commandAllMotors(){
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
}*/

//----------------------------------------------------
//    Stand UP function
//----------------------------------------------------
bool stand_up_fc()
{
	bool flag = false;

	for(int i = 1; i <= (LEG_NUMBER); i++)
    {
        epos_f->ActivateProfilePosition(i);
        epos_f->SetPositionProfile(i, 2, 12000, 12000);
    }

    float stand_up_consigna = 4.54;
    float t1_offset = 0.52;
    float t2_offset = 5.76;

    const float t1_pos_set = stand_up_consigna + t1_offset - 0.001;
    const float t2_pos_set = stand_up_consigna - t1_offset;//stand_up_consigna + t2_offset - 0.001;

    std::vector<int>  tripode_1 = {1, 4, 5};
    std::vector<int>  tripode_2 = {2, 3, 6};

    std::vector<float>  tripode_1_pos_actual = {abs(epos_f->GetPosition(tripode_1.at(0))), abs(epos_f->GetPosition(tripode_1.at(1))), abs(epos_f->GetPosition(tripode_1.at(2)))};
    std::vector<float>  tripode_2_pos_actual = {abs(epos_f->GetPosition(tripode_2.at(0))), abs(epos_f->GetPosition(tripode_2.at(1))), abs(epos_f->GetPosition(tripode_2.at(2)))};

    ROS_INFO("Pata 1: %f -- Pata 2: %f -- Pata 3: %f -- Pata 4: %f -- Pata 5: %f -- Pata 6: %f", tripode_1_pos_actual.at(0), tripode_2_pos_actual.at(0), tripode_1_pos_actual.at(1), tripode_2_pos_actual.at(1), tripode_1_pos_actual.at(2), tripode_2_pos_actual.at(2));

    bool tripode_1_consigna_alcanzada = false;
    bool tripode_2_consigna_alcanzada = false;

    if(!flag)
    {
    	for(int i = 1; i <= LEG_NUMBER; i++)
    	{
    		if((i % 2) == 0){
    			epos_f->MoveToPosition((i), stand_up_consigna, 	true,   true);	
    		} else
    			epos_f->MoveToPosition((i), -stand_up_consigna,	true,   true);	

    		//ROS_INFO("Primer movimiento");
    	}
    	
    	for(int i = 0; i < tripode_1.size(); i++)
    	{
    		tripode_1_pos_actual.at(i) = epos_f->GetPosition(tripode_1.at(i));//(fmod(epos_f->GetPosition(tripode_1.at(i)), (2*PI)));// - stand_up_consigna);
    		tripode_2_pos_actual.at(i) = epos_f->GetPosition(tripode_2.at(i));//(fmod(epos_f->GetPosition(tripode_2.at(i)), (2*PI)));// - stand_up_consigna);
    	}

    	ROS_INFO("Patas a origen");
    	ROS_INFO("Pata 1: %f -- Pata 2: %f -- Pata 3: %f -- Pata 4: %f -- Pata 5: %f -- Pata 6: %f", tripode_1_pos_actual.at(0), tripode_2_pos_actual.at(0), tripode_1_pos_actual.at(1), tripode_2_pos_actual.at(1), tripode_1_pos_actual.at(2), tripode_2_pos_actual.at(2));


    	for(int i = 0; i < tripode_1.size(); i++)
    	{
    		if((i % 2) == 0){
    			epos_f->MoveToPosition(tripode_1.at(i), -t1_offset, false, false);
    		}else 
    			epos_f->MoveToPosition(tripode_1.at(i), t1_offset,	false, false);

    		//ROS_INFO("Segundo movimiento");
    	}

    	for(int i = 0; i < tripode_2.size(); i++)
    	{
    		//ROS_INFO("Entro en el tercer movimiento");
    		if((i % 2) == 0){
    			epos_f->MoveToPosition(tripode_2.at(i), t2_offset, 	false, false);
    		}else
    			epos_f->MoveToPosition(tripode_2.at(i), -t2_offset,	false, false);

    		//ROS_INFO("Tercer movimiento");
    	}

    	/*while(1){
    	for(int i = 0; i < tripode_1.size(); i++)
    	{
    		tripode_1_pos_actual.at(i) = fabs(fmod(epos_f->GetPosition(tripode_1.at(i)), (2*PI))) - stand_up_consigna;
    		tripode_2_pos_actual.at(i) = fabs(fmod(epos_f->GetPosition(tripode_2.at(i)), (2*PI))) - stand_up_consigna;
    	}

    	ROS_INFO("Patas en angulos");
		ROS_INFO("Pata 1: %f -- Pata 2: %f -- Pata 3: %f -- Pata 4: %f -- Pata 5: %f -- Pata 6: %f", tripode_1_pos_actual.at(0), tripode_2_pos_actual.at(0), tripode_1_pos_actual.at(1), tripode_2_pos_actual.at(1), tripode_1_pos_actual.at(2), tripode_2_pos_actual.at(2));
			    }*/
	}

    while(!flag)
    {
    	for(int i = 0; i < tripode_1.size(); i++)
    	{
    		tripode_1_pos_actual.at(i) = fabs(fmod(epos_f->GetPosition(tripode_1.at(i)), (2*PI))) - stand_up_consigna;
    		tripode_2_pos_actual.at(i) = fabs(fmod(epos_f->GetPosition(tripode_2.at(i)), (2*PI))) - stand_up_consigna;
    	}
	    /*float pata_2_pos_actual = fabs(epos_f->GetPosition(2));
	    float pata_4_pos_actual = fabs(epos_f->GetPosition(4));
	    float pata_6_pos_actual = fabs(epos_f->GetPosition(6));*/
		ROS_INFO("While");
	    ROS_INFO("Pata 1: %f -- Pata 2: %f -- Pata 3: %f -- Pata 4: %f -- Pata 5: %f -- Pata 6: %f", tripode_1_pos_actual.at(0), tripode_2_pos_actual.at(0), tripode_1_pos_actual.at(1), tripode_2_pos_actual.at(1), tripode_1_pos_actual.at(2), tripode_2_pos_actual.at(2));

    	if(( tripode_1_pos_actual.at(0) >= (t1_offset - 0.01)) &&
    		(tripode_1_pos_actual.at(1) >= -(t1_offset + 0.01)) &&
    		(tripode_1_pos_actual.at(2) >= (t1_offset - 0.01)))// && (!tripode_1_consigna_alcanzada)) 
    	{
    		tripode_1_consigna_alcanzada = true;
    		ROS_INFO("Tripode 1 posicion alcanzada");
    	}

    	if(( tripode_2_pos_actual.at(0) >= -(t1_offset + 0.01)) &&
    		(tripode_2_pos_actual.at(1) >= (t1_offset - 0.01)) &&
    		(tripode_2_pos_actual.at(2) >= -(t1_offset + 0.01)))// && (!tripode_2_consigna_alcanzada))
    	{
    		tripode_2_consigna_alcanzada = true;
    		ROS_INFO("Tripode 2 posicion alcanzada");
    	}
	    
	    /*if((pata_2_pos_actual >= (t2_pos_set - 0.01)) &&
	       (pata_4_pos_actual >= (t1_pos_set)) &&
	       (pata_6_pos_actual >= (t2_pos_set - 0.01)))
	    {
	    	ROS_INFO("Posicion alcanzada");
	        flag = true;
	    }*/

		if((tripode_1_consigna_alcanzada == true) && (tripode_2_consigna_alcanzada == true))
		{
			flag = false;
			return true;
		}
	}

}

//----------------------------------------------------
//    Forward
//----------------------------------------------------
bool alternating_tripod_fc()
{

	// Variables para verificar las dos fases de movimiento de las patas
	bool fase_1 = false;
	bool fase_2 = true;

	// Constantes de desfase, angulos y velocidad
	const float stand_up_offset = 4.54;
	const float suelo_recorrido = 1.05;
	const float vuelo_recorrido = 5.24;
	const float vuelo_velocidad = 4;
	const float suelo_velocidad = 0.81;
	const float stand_up_consigna = 4.54;

	// Definicion de todos los vectores	
	std::vector<int>  tripode_1 = {1, 4, 5};
    std::vector<int>  tripode_2 = {2, 3, 6};
    std::vector<bool> tripode_1_posicion_vector = {false, false, false}; // Vector que almacena que cada pata ha llegado a su consigna
    std::vector<bool> tripode_2_posicion_vector = {false, false, false}; 
    
    bool tripode_1_consigna_enviada = false; // Variable que indica que se ha enviado la consigna de posicion a todas las patas del tripode
    bool tripode_2_consigna_enviada = false;
	std::vector<bool> tripode_1_consigna_enviada_vector = {false, false, false};
    std::vector<bool> tripode_2_consigna_enviada_vector = {false, false, false}; // Vector que almacena que se ha enviado la consigna de posicion a cada pata del tripode. Cuando es todo TRUE se activa tripode_2_consigna_enviada

    std::vector<float>  tripode_1_pos_actual = {epos_f->GetPosition(tripode_1.at(0)), abs(epos_f->GetPosition(tripode_1.at(1))), epos_f->GetPosition(tripode_1.at(2))};
    std::vector<float>  tripode_2_pos_actual = {epos_f->GetPosition(tripode_2.at(0)), abs(epos_f->GetPosition(tripode_2.at(1))), epos_f->GetPosition(tripode_2.at(2))};

    std::vector<float>  tripode_1_pos_old = {999, -999, 999};
    std::vector<float>  tripode_2_pos_old = {-999, 999, -999};

    std::vector<bool> tripode_1_consigna_alcanzada_vector = {false, false, false}; // Vector que almacena que cada pata ha alcanzado su consigna
    std::vector<bool> tripode_2_consigna_alcanzada_vector = {false, false, false}; 

    bool tripode_1_consigna_alcanzada = false; // Variable que almacena que todas las patas han alcanzado su consigna
    bool tripode_2_consigna_alcanzada = false; 

	std::vector<float> tripode_1_actual_pos;
	std::vector<float> tripode_2_actual_pos;


	/*float pata_2_actual_pos, pata_4_actual_pos;
	float pata_2_old_pos, pata_4_old_pos;

	bool pata_2_consigna_enviada = false;   //Variable para enviar una única vez la orden de movimiento
    bool pata_4_consigna_enviada = false;

    bool pata_2_posicion = false;
    bool pata_4_posicion = false;*/

    for(int i = 0; i < tripode_1.size(); i++)
   	{
   		tripode_1_pos_actual.at(i) = fabs(fmod(epos_f->GetPosition(tripode_1.at(i)), (2*PI))) - stand_up_consigna;
   		tripode_2_pos_actual.at(i) = fabs(fmod(epos_f->GetPosition(tripode_2.at(i)), (2*PI))) - stand_up_consigna;
	}

	ROS_INFO("alternating tripod");
    ROS_INFO("Pata 1: %f -- Pata 2: %f -- Pata 3: %f -- Pata 4: %f -- Pata 5: %f -- Pata 6: %f", tripode_1_pos_actual.at(0), tripode_2_pos_actual.at(0), tripode_1_pos_actual.at(1), tripode_2_pos_actual.at(1), tripode_1_pos_actual.at(2), tripode_2_pos_actual.at(2));

    //ROS_INFO("Pata 2 old = %f  -- Pata 4 old = %f", pata_2_old_pos, pata_4_old_pos);


	while(!fase_1 && fase_2)
    {
    	if(!tripode_2_consigna_enviada)
        {
            for(int i = 0; i < tripode_2.size(); i++)
            {
                // Envio la consigna de posicion y movimiento una única vez
                if(!tripode_2_consigna_enviada_vector.at(i))
                {
                    // Activo perfiles de velocidad de la fase
                    epos_f->SetPositionProfile(tripode_2.at(i), suelo_velocidad, 12000, 12000);
                    if(i == 1)
                        epos_f->MoveToPosition(tripode_2.at(i), -suelo_recorrido, false, true);
                    else
                        epos_f->MoveToPosition(tripode_2.at(i), suelo_recorrido, false, true);
            
                    tripode_2_consigna_enviada_vector.at(i) = true;
                    //pata_2_consigna_enviada = true;
                }
            }
            //compruebo que se ha enviado la consigna a todas las patas
            tripode_2_consigna_enviada = std::all_of(tripode_2_consigna_enviada_vector.begin(), tripode_2_consigna_enviada_vector.end(), [](bool tripode_2_posicion_enviada) {return tripode_2_posicion_enviada;});
            //ROS_INFO("Consignas enviadas");
        }
    	
		if(!tripode_1_consigna_enviada)
        {
            for(int i = 0; i < tripode_1.size(); i++)
            {
                // Envio la consigna de posicion y movimiento una única vez
                if(!tripode_1_consigna_enviada_vector.at(i))
                {
                    // Activo perfiles de velocidad de la fase
                    epos_f->SetPositionProfile(tripode_1.at(i), vuelo_velocidad, 12000, 12000);
                    if((i % 2) == 0){
                        epos_f->MoveToPosition(tripode_1.at(i), -vuelo_recorrido, false, true);
                    }else
                        epos_f->MoveToPosition(tripode_1.at(i), vuelo_recorrido, false, true);
                    tripode_1_consigna_enviada_vector.at(i) = true;
                    //pata_2_consigna_enviada = true;
                }
            }
            //compruebo que se ha enviado la consigna a todas las patas
            tripode_1_consigna_enviada = std::all_of(tripode_1_consigna_enviada_vector.begin(), tripode_1_consigna_enviada_vector.end(), [](bool tripode_1_posicion_enviada) {return tripode_1_posicion_enviada;});
        }

    	for(int i = 0; i < tripode_2.size(); i++)
            {
            	tripode_2_pos_actual.at(i) = fabs(fmod(epos_f->GetPosition(tripode_2.at(i)), (2*PI))) - stand_up_consigna;
                //tripode_2_pos_actual.at(i) = (abs(epos_f->GetPosition(tripode_2.at(i))) - tripode_2_desfase) % 360;
                //ROS_INFO("Posicion de la pata %d = %d -- Desfase = %d", tripode_2.at(i), tripode_2_pos_actual.at(i), mcd_epos.GetPosition(tripode_2.at(i)));
            }
            for(int i = 0; i < tripode_1.size(); i++)
            {
            	tripode_1_pos_actual.at(i) = fabs(fmod(epos_f->GetPosition(tripode_1.at(i)), (2*PI))) - stand_up_consigna;
                //tripode_1_pos_actual.at(i) = (abs(epos_f->GetPosition(tripode_1.at(i))) - tripode_1_desfase) % 360;
                //ROS_INFO("Posicion de la pata %d = %f ", tripode_1.at(i), tripode_1_pos_actual.at(i));//, epos_f->GetPosition(tripode_1.at(i)));
            }

        if(!tripode_2_consigna_alcanzada)
        {
            for(int i = 0; i < tripode_2.size(); i++)
            {
            	if(((tripode_2_pos_actual.at(i)) >= (suelo_recorrido/2)) && !tripode_2_consigna_alcanzada_vector.at(i))
                //if((tripode_2_pos_actual.at(i) >= (recorrido_suelo / 2)) && (tripode_2_pos_actual.at(i) < 300) && (!tripode_2_posicion_vector.at(i)))//(pata_2_old_pos + recorrido_suelo))
                {
                    epos_f->HaltPositionMovement(tripode_2.at(i));
                    ROS_INFO("La pata %d ha llegado a la posicion: %f", tripode_2.at(i), tripode_2_pos_actual.at(i));
                    tripode_2_pos_old.at(i) = tripode_2_pos_actual.at(i);
                    //pata_2_old_pos = pata_2_actual_pos % 360;
                    tripode_2_consigna_alcanzada_vector.at(i) = true;
                    //pata_2_posicion = true;
                }
            }
            tripode_2_consigna_alcanzada = std::all_of(tripode_2_consigna_alcanzada_vector.begin(), tripode_2_consigna_alcanzada_vector.end(), [](bool tripode_2_consigna_alcanzada_flag) {return tripode_2_consigna_alcanzada_flag;});
        }

        if(!tripode_1_consigna_alcanzada)
        {
            for(int i = 0; i < tripode_1.size(); i++)
            {
            	if((tripode_1_pos_actual.at(i) <= (-0.50)) && (tripode_1_pos_actual.at(i) >= (-0.53)) && !tripode_1_consigna_alcanzada_vector.at(i))
                //if((abs(tripode_1_pos_actual.at(i)) >= (recorrido_vuelo + 30)) && (!tripode_1_posicion_vector.at(i)))//(pata_2_old_pos + recorrido_suelo))
                {
                    epos_f->HaltPositionMovement(tripode_1.at(i));
                    ROS_INFO("La pata %d ha llegado a la posicion: %f", tripode_1.at(i), tripode_1_pos_actual.at(i));
                    tripode_1_pos_old.at(i) = tripode_1_pos_actual.at(i);
                    //pata_2_old_pos = pata_2_actual_pos % 360;
                    tripode_1_consigna_alcanzada_vector.at(i) = true;
                    //pata_2_posicion = true;
                }
            }
            tripode_1_consigna_alcanzada = std::all_of(tripode_1_consigna_alcanzada_vector.begin(), tripode_1_consigna_alcanzada_vector.end(), [](bool tripode_1_consigna_alcanzada_flag) {return tripode_1_consigna_alcanzada_flag;});
        }

        /**** Salida de la Fase 1 ****/
        if(tripode_1_consigna_alcanzada && tripode_2_consigna_alcanzada)//(tripode_2_consigna_alcanzada && pata_4_posicion)
        {
            ROS_INFO("FIN de fase 1");
            fase_1 = true;
            fase_2 = false;

            tripode_1_consigna_alcanzada = false;
            tripode_1_consigna_enviada = false;
            for(int i = 0; i < tripode_1.size(); i++)
            {
                tripode_1_consigna_enviada_vector.at(i) = false;
                tripode_1_consigna_alcanzada_vector.at(i) = false;
            }

            tripode_2_consigna_alcanzada = false;
            tripode_2_consigna_enviada = false;
            for(int i = 0; i < tripode_2.size(); i++)
            {
                tripode_2_consigna_enviada_vector.at(i) = false;
                tripode_2_consigna_alcanzada_vector.at(i) = false;
            }

            /*pata_2_posicion = false;
            pata_4_posicion = false;
            pata_2_consigna_enviada = false;
            pata_4_consigna_enviada = false;*/
        }
    } // while(!fase_1 && fase_2)
    	// Envio la consigna de posicion y movimiento una única vez
        /*if(!pata_2_consigna_enviada)
        {
            // Activo perfiles de velocidad de la fase
            epos_f->SetPositionProfile(2, suelo_velocidad, 12000, 12000);
            epos_f->MoveToPosition(2, suelo_recorrido, false, true);
            pata_2_consigna_enviada = true;
        }
        if(!pata_4_consigna_enviada)
        {
            // Activo perfiles de velocidad de la fase
            epos_f->SetPositionProfile(4, vuelo_velocidad, 12000, 12000);
            epos_f->MoveToPosition(4, vuelo_recorrido, false, true);
            pata_4_consigna_enviada = true;
        }

        // Compruebo que la pata ha llegado
        pata_2_actual_pos = (fmod(epos_f->GetPosition(2), (2*PI)) - stand_up_offset);
        pata_4_actual_pos = (fmod(epos_f->GetPosition(4), (2*PI)) - stand_up_offset);
        ROS_INFO("Pata 2 = %f  -- Pata 4 = %f", pata_2_actual_pos, pata_4_actual_pos);

        if((pata_2_actual_pos >= (suelo_recorrido/2)) && !pata_2_posicion)//((pata_2_actual_pos >= (suelo_recorrido/2)) && (pata_2_actual_pos < 5.24) && (!pata_2_posicion)); //((pata_2_actual_pos >= (suelo_recorrido / 2)) && //(pata_2_old_pos + recorrido_suelo))
        {
            epos_f->HaltPositionMovement(2);
            ROS_INFO("La pata 2 ha llegado a la posicion: %f", fmod(epos_f->GetPosition(2), (2*PI)));
            pata_2_old_pos = fmod(epos_f->GetPosition(2), (2*PI));
            pata_2_posicion = true;
        }	

        // La pata 4 debe de llegar a 330º
        if((pata_4_actual_pos < 0) && (pata_4_actual_pos >= (-(suelo_recorrido/2))) && (!pata_4_posicion)) //(vuelo_recorrido + (vuelo_recorrido/2))) && (!pata_4_posicion))//(pata_4_old_pos + recorrido_vuelo))
        {
            epos_f->HaltPositionMovement(4);
            ROS_INFO("La pata 4 ha llegado a la posicion: %f", fmod(epos_f->GetPosition(4), (2*PI)));
            pata_4_old_pos = pata_4_actual_pos;
            pata_4_posicion = true;
        }

        if(pata_2_posicion && pata_4_posicion)
    	{
    		ROS_INFO("FIN de fase 1");
            fase_1 = true;
            fase_2 = false;
            pata_2_posicion = false;
            pata_4_posicion = false;
            pata_2_consigna_enviada = false;
            pata_4_consigna_enviada = false;
        }
    }*/
    while(fase_1 && !fase_2)
    {

	    if(!tripode_2_consigna_enviada)
	        {
	            for(int i = 0; i < tripode_2.size(); i++)
	            {
	                // Envio la consigna de posicion y movimiento una única vez
	                if(!tripode_2_consigna_enviada_vector.at(i))
	                {
	                    // Activo perfiles de velocidad de la fase
	                    epos_f->SetPositionProfile(tripode_2.at(i), vuelo_velocidad, 12000, 12000);
	                    if(i == 1)
	                        epos_f->MoveToPosition(tripode_2.at(i), -vuelo_recorrido, false, true);
	                    else
	                        epos_f->MoveToPosition(tripode_2.at(i), vuelo_recorrido, false, true);
	            
	                    tripode_2_consigna_enviada_vector.at(i) = true;
	                    //pata_2_consigna_enviada = true;
	                }
	            }
	            //compruebo que se ha enviado la consigna a todas las patas
	            tripode_2_consigna_enviada = std::all_of(tripode_2_consigna_enviada_vector.begin(), tripode_2_consigna_enviada_vector.end(), [](bool tripode_2_posicion_enviada) {return tripode_2_posicion_enviada;});
	            //ROS_INFO("Consignas enviadas");
	        }
	    	
		if(!tripode_1_consigna_enviada)
	    {
	        for(int i = 0; i < tripode_1.size(); i++)
	        {
	            // Envio la consigna de posicion y movimiento una única vez
	            if(!tripode_1_consigna_enviada_vector.at(i))
	            {
	                // Activo perfiles de velocidad de la fase
	                epos_f->SetPositionProfile(tripode_1.at(i), suelo_velocidad, 12000, 12000);
	                if((i % 2) == 0){
	                    epos_f->MoveToPosition(tripode_1.at(i), -suelo_recorrido, false, true);
	                }else
	                    epos_f->MoveToPosition(tripode_1.at(i), suelo_recorrido, false, true);
	                tripode_1_consigna_enviada_vector.at(i) = true;
	                //pata_2_consigna_enviada = true;
	            }
	        }
	        //compruebo que se ha enviado la consigna a todas las patas
	        tripode_1_consigna_enviada = std::all_of(tripode_1_consigna_enviada_vector.begin(), tripode_1_consigna_enviada_vector.end(), [](bool tripode_1_posicion_enviada) {return tripode_1_posicion_enviada;});
	    }

		for(int i = 0; i < tripode_2.size(); i++)
	    {
	        	tripode_2_pos_actual.at(i) = fabs(fmod(epos_f->GetPosition(tripode_2.at(i)), (2*PI))) - stand_up_consigna;
	            //tripode_2_pos_actual.at(i) = (abs(epos_f->GetPosition(tripode_2.at(i))) - tripode_2_desfase) % 360;
	        //ROS_INFO("Posicion de la pata %d = %d -- Desfase = %d", tripode_2.at(i), tripode_2_pos_actual.at(i), mcd_epos.GetPosition(tripode_2.at(i)));
		}
	    for(int i = 0; i < tripode_1.size(); i++)
	    {
	    	tripode_1_pos_actual.at(i) = fabs(fmod(epos_f->GetPosition(tripode_1.at(i)), (2*PI))) - stand_up_consigna;
	        //tripode_1_pos_actual.at(i) = (abs(epos_f->GetPosition(tripode_1.at(i))) - tripode_1_desfase) % 360;
	        //ROS_INFO("Posicion de la pata %d = %f -- Desfase = %f", tripode_1.at(i), tripode_1_pos_actual.at(i), epos_f->GetPosition(tripode_1.at(i)));
	    }

	    if(!tripode_2_consigna_alcanzada)
	    {
	        for(int i = 0; i < tripode_2.size(); i++)
	        {
	        	if((tripode_2_pos_actual.at(i) <= (-0.50)) && (tripode_2_pos_actual.at(i) >= (-0.53)) && !tripode_2_consigna_alcanzada_vector.at(i))
	            //if((tripode_2_pos_actual.at(i) >= (recorrido_suelo / 2)) && (tripode_2_pos_actual.at(i) < 300) && (!tripode_2_posicion_vector.at(i)))//(pata_2_old_pos + recorrido_suelo))
	            {
	                epos_f->HaltPositionMovement(tripode_2.at(i));
	                ROS_INFO("La pata %d ha llegado a la posicion: %f", tripode_2.at(i), tripode_2_pos_actual.at(i));
	                tripode_2_pos_old.at(i) = tripode_2_pos_actual.at(i);
	                //pata_2_old_pos = pata_2_actual_pos % 360;
	                tripode_2_consigna_alcanzada_vector.at(i) = true;
	                //pata_2_posicion = true;
	            }
	        }
	        tripode_2_consigna_alcanzada = std::all_of(tripode_2_consigna_alcanzada_vector.begin(), tripode_2_consigna_alcanzada_vector.end(), [](bool tripode_2_consigna_alcanzada_flag) {return tripode_2_consigna_alcanzada_flag;});
    		if(tripode_2_consigna_alcanzada)
        		ROS_INFO("Tripode 2 consigna alcanzada");
	    }

	    if(!tripode_1_consigna_alcanzada)
	    {
	        for(int i = 0; i < tripode_1.size(); i++)
	        {
	        	if((tripode_1_pos_actual.at(i) >= (suelo_recorrido/2)) && !tripode_1_consigna_alcanzada_vector.at(i))
	            //if((abs(tripode_1_pos_actual.at(i)) >= (recorrido_vuelo + 30)) && (!tripode_1_posicion_vector.at(i)))//(pata_2_old_pos + recorrido_suelo))
	            {
	                epos_f->HaltPositionMovement(tripode_1.at(i));
	                ROS_INFO("La pata %d ha llegado a la posicion: %f", tripode_1.at(i), tripode_1_pos_actual.at(i));
	                tripode_1_pos_old.at(i) = tripode_1_pos_actual.at(i);
	                //pata_2_old_pos = pata_2_actual_pos % 360;
	                tripode_1_consigna_alcanzada_vector.at(i) = true;
	                //pata_2_posicion = true;
	            }
	        }
	        tripode_1_consigna_alcanzada = std::all_of(tripode_1_consigna_alcanzada_vector.begin(), tripode_1_consigna_alcanzada_vector.end(), [](bool tripode_1_consigna_alcanzada_flag) {return tripode_1_consigna_alcanzada_flag;});
	        if(tripode_1_consigna_alcanzada)
        		ROS_INFO("Tripode 1 consigna alcanzada");
	    }

	    /**** Salida de la Fase 2 ****/
	    if(tripode_1_consigna_alcanzada && tripode_2_consigna_alcanzada)//(tripode_2_consigna_alcanzada && pata_4_posicion)
	    {
	        ROS_INFO("FIN de fase 2");
	        fase_1 = false;
	        fase_2 = true;

	        tripode_1_consigna_alcanzada = false;
	        tripode_1_consigna_enviada = false;
	        for(int i = 0; i < tripode_1.size(); i++)
	        {
	            tripode_1_consigna_enviada_vector.at(i) = false;
	            tripode_1_consigna_alcanzada_vector.at(i) = false;
	        }

	        tripode_2_consigna_alcanzada = false;
	        tripode_2_consigna_enviada = false;
	        for(int i = 0; i < tripode_2.size(); i++)
	        {
	            tripode_2_consigna_enviada_vector.at(i) = false;
	            tripode_2_consigna_alcanzada_vector.at(i) = false;
	        }

	        /*pata_2_posicion = false;
	        pata_4_posicion = false;
	        pata_2_consigna_enviada = false;
	        pata_4_consigna_enviada = false;*/
	    }
	}


	/*while(fase_1 && !fase_2)
    {
    	if(!pata_2_consigna_enviada)
        {
            // Activo perfiles de velocidad de la fase
            epos_f->SetPositionProfile(2, vuelo_velocidad, 12000, 12000);
            epos_f->MoveToPosition(2, vuelo_recorrido, false, true);
            pata_2_consigna_enviada = true;
        }
        if(!pata_4_consigna_enviada)
        {
            // Activo perfiles de velocidad de la fase
            epos_f->SetPositionProfile(4, suelo_velocidad, 12000, 12000);
            epos_f->MoveToPosition(4, suelo_recorrido, false, true);
            pata_4_consigna_enviada = true;
        }

        pata_2_actual_pos = (fmod(epos_f->GetPosition(2), (2*PI)) - stand_up_offset);
        pata_4_actual_pos = (fmod(epos_f->GetPosition(4), (2*PI)) - stand_up_offset);
        ROS_INFO("Pata 2 = %f  -- Pata 4 = %f", pata_2_actual_pos, pata_4_actual_pos);

        if((pata_2_actual_pos < 0) && (pata_2_actual_pos >= (-(suelo_recorrido/2))) && (!pata_2_posicion)) //(vuelo_recorrido + (vuelo_recorrido/2))) && (!pata_4_posicion))//(pata_4_old_pos + recorrido_vuelo))
        {
            epos_f->HaltPositionMovement(2);
            ROS_INFO("La pata 2 ha llegado a la posicion: %f", fmod(epos_f->GetPosition(2), (2*PI)));
            pata_2_old_pos = pata_2_actual_pos;
            pata_2_posicion = true;
        }

        if((pata_4_actual_pos >= (suelo_recorrido/2)) && !pata_4_posicion)//((pata_2_actual_pos >= (suelo_recorrido/2)) && (pata_2_actual_pos < 5.24) && (!pata_2_posicion)); //((pata_2_actual_pos >= (suelo_recorrido / 2)) && //(pata_2_old_pos + recorrido_suelo))
        {
            epos_f->HaltPositionMovement(4);
            ROS_INFO("La pata 4 ha llegado a la posicion: %f", fmod(epos_f->GetPosition(4), (2*PI)));
            pata_4_old_pos = fmod(epos_f->GetPosition(4), (2*PI));
            pata_4_posicion = true;
        }

        if(pata_2_posicion && pata_4_posicion)
    	{
    		ROS_INFO("FIN de fase 2");
            fase_1 = false;
            fase_2 = false;
            pata_2_posicion = false;
            pata_4_posicion = false;
            pata_2_consigna_enviada = false;
            pata_4_consigna_enviada = false;
        }
    }*/
	

	//return true;
}
//----------------------------------------------------
//    Right
//----------------------------------------------------
bool turn_right_fc()
{
	return true;	
}
//----------------------------------------------------
//    Left
//----------------------------------------------------
bool turn_left_fc()
{
	return true;
}
//----------------------------------------------------
//    Get Down
//----------------------------------------------------
bool get_down_fc()
{
	return true;
}


int main(int argc, char **argv){

  //----------------------------------------------------
  //    ROS starting statements
  //----------------------------------------------------

  ros::init(argc, argv, "clhero_hardware_interface");
  ros::NodeHandle nh;

  ros::Rate loop_rate (LOOP_RATE);

  //Creates the maxon motors'handler
  epos_f = new epos_functions();

  //Sets the default profile
  /*for(int i = 0; i < LEG_NUMBER; i++){
  	epos_f->ActivateProfilePosition(mapMotor(i+1));
  	epos_f->SetPositionProfile(mapMotor(i+1), DEFAULT_VEL, DEFAULT_ACCEL, DEFAULT_DECEL);
  	//epos_f->SetVelocityProfile(mapMotor(i+1), DEFAULT_ACCEL, DEFAULT_DECEL);
  }*/
  epos_f->ActivateProfilePosition(1);
  epos_f->SetPositionProfile(1, DEFAULT_VEL, DEFAULT_ACCEL, DEFAULT_DECEL);

  //Creates the msg manager
  //com_man = new CommandMsgManager(epos_f);

  //Publishers
  legs_state_pub = nh.advertise<clhero_gait_controller::LegState>("legs_state", 1);
  
  //Topics subscription
  ros::Subscriber leg_command_sub = nh.subscribe("legs_command", 1, legCommandCallback);
  ros::Subscriber pattern_command_sub = nh.subscribe("pattern_command", 1, patternCommandCallback);

  //ros::Subscriber joint_states_sub = nh.subscribe("/hexapodo/joint_states", 1000, jointStatesCallback);
  
  //threads which helds the status publishing function
  //std::thread state_update_thr (StateUpdateThread);

  bool stand_up_flag = false;	// El robot está en el suelo

  //----------------------------------------------------
  //    Core loop of the node
  //----------------------------------------------------
  while(ros::ok()){

  	//switch(pattern_mode_number):
  	if(!stand_up_flag)
  	{
  		//ROS_INFO("Espero a levantarme");
  		if((pattern_mode_number == 10) && (pattern_state_number == 1))
  		{
  			ROS_INFO("Empiezo a levantarme");
  			stand_up_flag = stand_up_fc();
  			ROS_INFO("He salido");
  			//stand_up_flag = true;
  		}
  	}

  	else
  	{
  		static bool msg_flag = false;
  		if(!msg_flag)
  		{
  			ROS_INFO("Estoy levantado");
  			msg_flag = true;
  		}

  		static bool flag = false;

  		switch(pattern_mode_number){
  			// FORWARD
  			case 1:
  				
  				if(pattern_state_number == 1)
  				{
  					flag = alternating_tripod_fc();
  					ROS_INFO("Forward");	
  				}
				if(flag)
					break;

			// RIGHT
			case 2:
				
				if(pattern_state_number == 1)
				{
  					flag = turn_right_fc();
  					ROS_INFO("Right");
				}
				if(flag)
					break;

			// LEFT
			case -2:
				
				if(pattern_state_number == 1)
				{
  					flag = turn_left_fc();
  					ROS_INFO("Left");
				}
				if(flag)
					break;

			// GET DOWN
			case -10:
				
				if(pattern_state_number == 1)
				{
					ROS_INFO("Get Down");
  					flag = get_down_fc();
  				}
				if(flag)
					break;

  		}

  	}

  	ros::spinOnce();
  	loop_rate.sleep();	
  }

  //----------------------------------------------------
  //    End of node statements
  //----------------------------------------------------

  //state_update_thr.join();

  //delete epos_f;
  //delete com_man;

  return 0;

}
