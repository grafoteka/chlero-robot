#include <ros/ros.h>
//#include <std_msgs/String.h>
//#include <altitude_sensor/sensor_data.h>
//#include <cereal_port/CerealPort.h>
//#include <altitude_sensor/Definitions.h>
//#include <sensor_msgs/JointState.h>
#include <epos_functions/epos_functions.h>

ros::Publisher joint_states_pub;

#define REPLY_SIZE 20
#define TIMEOUT 1000

const unsigned int sensor_frequency = 10; /*sensor frequency in hz*/

double altitude;
double voltage;
//std::string serial_port;

typedef void* HANDLE;
typedef int BOOL;

using namespace std;

// Vector para guardar el offset inicial de las patas.
double motor_initial_offset_[6];



// Esta función debo de llamarla cuando cierre el programa.
// Está todavía pendiente de ser implementada
/*int CloseDevice(unsigned int* p_pErrorCode, int node_id)
{
        int lResult = MMC_FAILED;

        *p_pErrorCode = 0;

        LogInfo("Close device");

        if(VCS_CloseDevice(g_pKeyHandle, p_pErrorCode)!=0 && *p_pErrorCode == 0)
        {
                lResult = MMC_SUCCESS;
        }

        return lResult;
}*/

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
    //joint_states_pub = nh.advertise<sensor_msgs::JointState>("hexapodo_joint_states", 1000);

    ros::Rate loop_rate(sensor_frequency);

    epos_functions mcd_epos;

    int error = 1;

    std::vector<int> lado_derecho = {1, 3, 5};
    std::vector<int> lado_izquierdo = {2, 4, 6};

    std::vector<int> tripode_1 = {1, 4, 5};
    std::vector<int> tripode_2 = {2, 3, 6};

    int tripode_1_vel = 400;
    int tripode_2_vel = 400;
    // Activo los perfiles de posicion y les doy un perfil de velocidad
    // Lado derecho

    for(int i = 0; i < tripode_1.size(); i++)
    {
        mcd_epos.ActivateProfilePosition(tripode_1.at(i));
        mcd_epos.SetPositionProfile(tripode_1.at(i), tripode_1_vel, 12000, 12000);
    }
    for(int i = 0; i < tripode_2.size(); i++)
    {
        mcd_epos.ActivateProfilePosition(tripode_2.at(i));
        mcd_epos.SetPositionProfile(tripode_2.at(i), tripode_2_vel, 12000, 12000);
    }

    /*mcd_epos.MoveToPosition(1, -260, false, true);
    mcd_epos.MoveToPosition(2,  260, false, true);
    mcd_epos.MoveToPosition(3, -260, false, true);
    mcd_epos.MoveToPosition(4,  260, false, true);
    mcd_epos.MoveToPosition(5, -260, false, true);
    mcd_epos.MoveToPosition(6,  260, false, true);*/

    //mcd_epos.SetPositionProfile(4, 945.45, 12000, 12000);

    //result = mcd_epos.MoveToPosition(6, 132000, false, true);

    int stand_up_consigna = 260;
    int desfase = 0;

    std::vector<int> tripode_1_consigna = {-(stand_up_consigna - desfase), stand_up_consigna - desfase, -(stand_up_consigna - desfase)};
    std::vector<int> tripode_2_consigna = {stand_up_consigna + desfase, -(stand_up_consigna + desfase), stand_up_consigna + desfase};

    std::vector<int> consignas = {tripode_1_consigna.at(0), tripode_2_consigna.at(0),
                                  -tripode_1_consigna.at(1), -tripode_2_consigna.at(1),
                                  tripode_1_consigna.at(2), tripode_2_consigna.at(2)};
    // tripode 1 = consigna - desfase;
    // tripode 2 = consigna + desfase;

    //mcd_epos.MoveToPosition(1, 260, false, true);

    for(int i = 1; i <= 6; i++)
    {
        mcd_epos.MoveToPosition(i, consignas.at(i-1), false, true);
        //ROS_INFO(" Pata %d consigna %d", i, consignas.at(i-1));
    }

    // Poner en posicion vertical
    while(1)
    {
        int pata_1_pos_actual = abs(mcd_epos.GetPosition(1));
        int pata_2_pos_actual = abs(mcd_epos.GetPosition(2));
        int pata_3_pos_actual = abs(mcd_epos.GetPosition(3));
        int pata_4_pos_actual = abs(mcd_epos.GetPosition(4));
        int pata_5_pos_actual = abs(mcd_epos.GetPosition(5));
        int pata_6_pos_actual = abs(mcd_epos.GetPosition(6));
        /*ROS_INFO("%d -- %d -- %d -- %d -- %d -- %d",
                pata_1_pos_actual, pata_2_pos_actual,
                pata_3_pos_actual, pata_4_pos_actual,
                pata_5_pos_actual, pata_6_pos_actual);*/

        if((pata_1_pos_actual == (stand_up_consigna - 1)) && (pata_2_pos_actual == (stand_up_consigna - 1)) &&
           (pata_3_pos_actual == (stand_up_consigna - 1)) && (pata_4_pos_actual == (stand_up_consigna - 1)) &&
           (pata_5_pos_actual == (stand_up_consigna - 1)) && (pata_6_pos_actual == (stand_up_consigna - 1)))
        {
            break;
        }
    }

    // Mover el tripode 2 ->
    while(1)
    {
        int pata_2_pos_actual = abs(mcd_epos.GetPosition(2));
        int pata_3_pos_actual = abs(mcd_epos.GetPosition(3));
        int pata_6_pos_actual = abs(mcd_epos.GetPosition(6));

        static bool orden = false;
        if(!orden)
        {
	    const int tripode_2_vel_stand_up = 750;
	    for(int i = 0; i < tripode_2.size(); i++)
	    {
		mcd_epos.SetPositionProfile(tripode_2.at(i), tripode_2_vel_stand_up, 12000, 12000);
	    }
            //ROS_INFO("Doy consigna");
            mcd_epos.MoveToPosition(2, 330, false, true);
            mcd_epos.MoveToPosition(3, -330, false, true);
            mcd_epos.MoveToPosition(6, 330, false, true);
            orden = true;
        }

        /*ROS_INFO("%d -- %d -- %d",
                pata_2_pos_actual, pata_3_pos_actual, pata_6_pos_actual);*/

        if((pata_2_pos_actual == (stand_up_consigna + 330 - 1)) &&
           (pata_3_pos_actual == (stand_up_consigna + 330 - 1)) &&
           (pata_6_pos_actual == (stand_up_consigna + 330 - 1)))
        {
            break;
        }
    }

    // Mover el tripode 1 -> stand up + 345 grados
    while(1)
    {
        int pata_1_pos_actual = abs(mcd_epos.GetPosition(1));
        int pata_4_pos_actual = abs(mcd_epos.GetPosition(4));
        int pata_5_pos_actual = abs(mcd_epos.GetPosition(5));

        static bool orden = false;
        if(!orden)
        {
            //ROS_INFO("Doy consigna");
            mcd_epos.MoveToPosition(1, -30, false, true);
            mcd_epos.MoveToPosition(4, 30, false, true);
            mcd_epos.MoveToPosition(5, -30, false, true);
            orden = true;
        }

        /*ROS_INFO("%d -- %d -- %d",
                pata_1_pos_actual, pata_4_pos_actual, pata_5_pos_actual);*/

        if((pata_1_pos_actual == (stand_up_consigna + 30 - 1)) &&
           (pata_4_pos_actual == (stand_up_consigna + 30 - 1)) &&
           (pata_5_pos_actual == (stand_up_consigna + 30 - 1)))
        {
            break;
        }
    }

    // GIRO

    /******************************
     *
     * T1 = pos_despegue
     * T2 = pos_aterrizaje
     * Transition_1 = T1 barrido & T2 vuelo
     * Transition_2 = T1 vuelo   & T2 barrido
     *
     * Barrido: 60  grados, 200 rpm
     * Vuelo:   300 grados, 800 rpm
     *
     * T1 posicion inicial:    -289
     * T1 posicion despegue:   -289
     * T1 posicion aterrizaje:   11
     * T2 posicion inicial:     589 -> 229
     * T2 posicion despegue:    649
     * T2 posicion aterrizaje:  589 -> 229
     *******************************/
/*    const int accel_max   = 12000;
    const int decel_max   = 12000;
    const int vel_aire    = 1000;
    const int vel_suelo   = 400;
    const int ang_barrido = 60;
    const int ang_vuelo   = 300;

    std::vector<int> posiciones_patas = {0, 0, 0, 0, 0, 0};
    std::vector<int> posiciones_patas_inicio = {0, 0, 0, 0, 0, 0};

    posiciones_patas_inicio.at(0) = (mcd_epos.GetPosition(tripode_1.at(0)) % 360);
    posiciones_patas_inicio.at(1) = (mcd_epos.GetPosition(tripode_2.at(0)) % 360);
    posiciones_patas_inicio.at(4) = (mcd_epos.GetPosition(tripode_1.at(2)) % 360);
    posiciones_patas_inicio.at(5) = (mcd_epos.GetPosition(tripode_2.at(2)) % 360);

    ROS_INFO("Posicion de las patas: %d -- %d -- %d -- %d", posiciones_patas.at(0), posiciones_patas.at(1), posiciones_patas.at(4), posiciones_patas.at(5));

    while(1)
    {
	static bool transition_1 = false;
	static bool transition_2 = true;
	static bool transition_1_order = false;
	static bool transition_2_order = false;
	
	static int counter = 0;
	// Giro derecha
	if(counter < 7)
	{
	// Lectura posicion patas
	//posiciones_patas.at(0) = mcd_epos.GetPosition(tripode_1.at(0));
    	//posiciones_patas.at(1) = mcd_epos.GetPosition(tripode_2.at(0));
    	//posiciones_patas.at(4) = mcd_epos.GetPosition(tripode_1.at(2));
    	//posiciones_patas.at(5) = mcd_epos.GetPosition(tripode_2.at(2));

	// Transition_1
	if(!transition_1 && transition_2 && !transition_1_order)
	{
	    ROS_INFO("Transicion 1");
	    // Configuracion MCDs
	    //mcd_epos.SetPositionProfile(1, vel_aire,  accel_max, decel_max);
	    //mcd_epos.SetPositionProfile(5, vel_aire,  accel_max, decel_max);
	    mcd_epos.SetPositionProfile(2, vel_suelo, accel_max, decel_max);
	    mcd_epos.SetPositionProfile(6, vel_suelo, accel_max, decel_max);

	    // Consignas
	    //mcd_epos.MoveToPosition(1, ang_barrido, false, true);
	    //mcd_epos.MoveToPosition(5, ang_barrido, false, true);
	    mcd_epos.MoveToPosition(2, ang_barrido, false, true);
	    mcd_epos.MoveToPosition(6, ang_barrido, false, true);

	    mcd_epos.SetPositionProfile(2, vel_aire, accel_max, decel_max);
	    mcd_epos.SetPositionProfile(6, vel_aire, accel_max, decel_max);

	    mcd_epos.MoveToPosition(2, ang_vuelo, false, false);
	    mcd_epos.MoveToPosition(6, ang_vuelo, false, false);

	    // Salida de la condicion
	    transition_1_order = true;
	    //transition_2 = false;
	}

	// Transition_2
	if(transition_1 && !transition_2 && !transition_2_order)
	{
	    ROS_INFO("Transicion 2");
	    // Configuracion MCDs
	    mcd_epos.SetPositionProfile(1, vel_suelo,  accel_max, decel_max);
	    mcd_epos.SetPositionProfile(5, vel_suelo,  accel_max, decel_max);

	    // Consignas
	    mcd_epos.MoveToPosition(1, ang_barrido,   false, true);
	    mcd_epos.MoveToPosition(5, ang_barrido,   false, true);
	    
	    mcd_epos.SetPositionProfile(1, vel_aire,  accel_max, decel_max);
	    mcd_epos.SetPositionProfile(5, vel_aire,  accel_max, decel_max);
	    
	    mcd_epos.MoveToPosition(1, ang_vuelo, false, false);
	    mcd_epos.MoveToPosition(5, ang_vuelo, false, false);

	    // Salida de la condicion
	    transition_2_order = true;
	    //transition_1 = false;
	}

	//Lectura posiciones
	if(!transition_1 && transition_1_order)// && !transition_2)
	{
	    
	    //posiciones_patas.at(0) = (mcd_epos.GetPosition(tripode_1.at(0)) % 360);
    	    posiciones_patas.at(1) = (mcd_epos.GetPosition(tripode_2.at(0)) );
    	    //posiciones_patas.at(4) = (mcd_epos.GetPosition(tripode_1.at(2)) % 360);
    	    posiciones_patas.at(5) = (mcd_epos.GetPosition(tripode_2.at(2)) );

    	    //ROS_INFO("Posicion de las patas: %d -- %d -- %d -- %d", posiciones_patas.at(0), posiciones_patas.at(1), posiciones_patas.at(4), posiciones_patas.at(5));

	    if(((posiciones_patas.at(1) % 360) == 229) && ((posiciones_patas.at(5) % 360) == 229))//>= (posiciones_patas_inicio.at(1) + 359)) && ((posiciones_patas.at(5) % 360) >= (posiciones_patas_inicio.at(5) + 359)))
	    {
		ROS_INFO("Detencion 1");
		transition_1 = true;
		transition_1_order = false;
		transition_2 = false;
	    }
	}

	//Lectura posiciones
	if(!transition_2 && transition_2_order)// && !transition_1)
	{
	    
	    posiciones_patas.at(0) = (mcd_epos.GetPosition(tripode_1.at(0)) % 360);
    	    //posiciones_patas.at(1) = (mcd_epos.GetPosition(tripode_2.at(0)) % 360);
    	    posiciones_patas.at(4) = (mcd_epos.GetPosition(tripode_1.at(2)) % 360);
    	    //posiciones_patas.at(5) = (mcd_epos.GetPosition(tripode_2.at(2)) % 360);

    	    //ROS_INFO("Posicion de las patas: %d -- %d -- %d -- %d", posiciones_patas.at(0), posiciones_patas.at(1), posiciones_patas.at(4), posiciones_patas.at(5));

	    if(((posiciones_patas.at(0) % 360) == (posiciones_patas_inicio.at(0) + 359)) && ((posiciones_patas.at(4) % 360) == (posiciones_patas_inicio.at(4) + 359)))
	    {
		ROS_INFO("Detencion 2");
		transition_2 = true;
		transition_2_order = false;
		transition_1 = false;
		counter++;
		ROS_INFO("Counter = %d", counter);
	    }
	}
	} // if (counter < 6)

	if((counter >= 7) && (counter < 15))
	{
	    // Transicion 1
	    if(!transition_1 && transition_2 && !transition_1_order)
	    {
	    	ROS_INFO("Giro Izquierda. Transicion 1");
		//posiciones_patas.at(0) = (mcd_epos.GetPosition(tripode_1.at(0)) % 360);
    	        //posiciones_patas.at(1) = (mcd_epos.GetPosition(tripode_2.at(0)) % 360);
       	        //posiciones_patas.at(4) = (mcd_epos.GetPosition(tripode_1.at(2)) % 360);
    	        //posiciones_patas.at(5) = (mcd_epos.GetPosition(tripode_2.at(2)) % 360);

    	        //ROS_INFO("Posicion de las patas: %d -- %d -- %d -- %d", posiciones_patas.at(0), posiciones_patas.at(1), posiciones_patas.at(4), posiciones_patas.at(5));

		// Configuracion MCDs
	        mcd_epos.SetPositionProfile(1, vel_aire,  accel_max, decel_max);
		mcd_epos.SetPositionProfile(5, vel_aire,  accel_max, decel_max);
		mcd_epos.MoveToPosition(1, -ang_vuelo,   false, true);
		mcd_epos.MoveToPosition(5, -ang_vuelo,   false, true);
		mcd_epos.SetPositionProfile(1, vel_suelo,  accel_max, decel_max);
		mcd_epos.SetPositionProfile(5, vel_suelo,  accel_max, decel_max);
		mcd_epos.MoveToPosition(1, -ang_barrido,   false, false);
		mcd_epos.MoveToPosition(5, -ang_barrido,   false, false);
	        //mcd_epos.SetPositionProfile(5, vel_suelo,  accel_max, decel_max);
		transition_1_order = true;
	    } // Transicion 1

	    // Transicion 2
	    if(transition_1 && !transition_2 && !transition_2_order)
	    {
		ROS_INFO("Giro Izquierda. Transicion 2");
		posiciones_patas.at(0) = (mcd_epos.GetPosition(tripode_1.at(0)) % 360);
    	        posiciones_patas.at(1) = (mcd_epos.GetPosition(tripode_2.at(0)) % 360);
       	        posiciones_patas.at(4) = (mcd_epos.GetPosition(tripode_1.at(2)) % 360);
    	        posiciones_patas.at(5) = (mcd_epos.GetPosition(tripode_2.at(2)) % 360);

    	        ROS_INFO("Posicion de las patas: %d -- %d -- %d -- %d", posiciones_patas.at(0), posiciones_patas.at(1), posiciones_patas.at(4), posiciones_patas.at(5));
		// Configuracion MCDs
	        mcd_epos.SetPositionProfile(2, vel_aire,  accel_max, decel_max);
		mcd_epos.SetPositionProfile(6, vel_aire,  accel_max, decel_max);
		mcd_epos.MoveToPosition(2, -ang_vuelo,   false, true);
		mcd_epos.MoveToPosition(6, -ang_vuelo,   false, true);
		mcd_epos.SetPositionProfile(2, vel_suelo,  accel_max, decel_max);
		mcd_epos.SetPositionProfile(6, vel_suelo,  accel_max, decel_max);
		mcd_epos.MoveToPosition(2, -ang_barrido,   false, false);
		mcd_epos.MoveToPosition(6, -ang_barrido,   false, false);
	        //mcd_epos.SetPositionProfile(5, vel_suelo,  accel_max, decel_max);
		transition_2_order = true;
	    } // Transicion 2

	    //Lectura posiciones
	    if(!transition_1 && transition_1_order)// && !transition_2)
	    {
	    	//ROS_INFO("Lectura de posiciones 1");
	    	posiciones_patas.at(0) = abs((mcd_epos.GetPosition(tripode_1.at(0)) % 360));
    	    	//posiciones_patas.at(1) = (mcd_epos.GetPosition(tripode_2.at(0)) );
    	    	posiciones_patas.at(4) = abs((mcd_epos.GetPosition(tripode_1.at(2)) % 360));
    	    	//posiciones_patas.at(5) = (mcd_epos.GetPosition(tripode_2.at(2)) );

    	    	ROS_INFO("Posicion de las patas: %d -- %d -- %d -- %d", posiciones_patas.at(0), posiciones_patas.at(1), posiciones_patas.at(4), posiciones_patas.at(5));

	    	if((((posiciones_patas.at(0) % 360) == 70) && ((posiciones_patas.at(4) % 360) == 70)) || ((((posiciones_patas.at(0) % 360) == 289) && ((posiciones_patas.at(4) % 360) == 289))))//>= (posiciones_patas_inicio.at(1) + 359)) && ((posiciones_patas.at(5) % 360) >= (posiciones_patas_inicio.at(5) + 359)))
	    	{
		    ROS_INFO("Detencion 1");
		    transition_1 = true;
		    transition_1_order = false;
		    transition_2 = false;
	        } //if(((posiciones_patas.at(1) % 360) == 229) && ((posiciones_patas.at(5) % 360) == 229))
	    }  //Lectura posiciones

	    //Lectura posiciones 2
	    if(!transition_2 && transition_2_order)// && !transition_2)
	    {
	    
	    	//posiciones_patas.at(0) = (mcd_epos.GetPosition(tripode_1.at(0)) % 360);
    	    	posiciones_patas.at(1) = (mcd_epos.GetPosition(tripode_2.at(0)) % 360);
    	    	//posiciones_patas.at(4) = (mcd_epos.GetPosition(tripode_1.at(2)) % 360);
    	    	posiciones_patas.at(5) = (mcd_epos.GetPosition(tripode_2.at(2)) % 360);

    	    	ROS_INFO("Posicion de las patas: %d -- %d -- %d -- %d", posiciones_patas.at(0), posiciones_patas.at(1), posiciones_patas.at(4), posiciones_patas.at(5));

	    	if(((posiciones_patas.at(1) % 360) == 229) && ((posiciones_patas.at(5) % 360) == 229))//>= (posiciones_patas_inicio.at(1) + 359)) && ((posiciones_patas.at(5) % 360) >= (posiciones_patas_inicio.at(5) + 359)))
	    	{
		    ROS_INFO("Detencion 2");
		    transition_2 = true;
		    transition_2_order = false;
		    transition_1 = false;
		    counter++;
		    ROS_INFO("Counter = %d", counter);
	        } //if(((posiciones_patas.at(1) % 360) == 229) && ((posiciones_patas.at(5) % 360) == 229))
	    }  //Lectura posiciones 2

	    //posiciones_patas.at(0) = (mcd_epos.GetPosition(tripode_1.at(0)) % 360);
	    //ROS_INFO("Position pata 1 = %d", posiciones_patas.at(0));
	    
	}
	 
    }
*/
    /*enum tripode_2_estados {aterrizaje, despegue, vuelo, suelo};
    tripode_2_estados tripode_2_estado_actual = aterrizaje;

    ROS_INFO("Empiezo a girar");
    int tripode_2_pos_actual = 0;
    int tripode_2_pos_old    = -999;
    int tripode_1_pos_actual = 0;
    int tripode_1_pos_old    = -999;

    for(int i = 0; i < tripode_1.size(); i++)
    {
	mcd_epos.SetPositionProfile(tripode_1.at(i), 400, 12000, 12000);
    }
    for(int i = 0; i < tripode_2.size(); i++)
    {
        mcd_epos.SetPositionProfile(tripode_2.at(i), 400, 12000, 12000);
    }
    tripode_1_pos_actual = abs(mcd_epos.GetPosition(1));
    tripode_1_pos_old    = (mcd_epos.GetPosition(1));
    ROS_INFO("tripode_1_pos_old = %d -- %d", tripode_1_pos_old, tripode_1_pos_old % 360);
    tripode_2_pos_actual = abs(mcd_epos.GetPosition(2));
    tripode_2_pos_old    = abs(mcd_epos.GetPosition(2));
*/
/*    pos_actual = abs(mcd_epos.GetPosition(2) % 360);
    ROS_INFO("Pos_actual = %d", pos_actual);
    mcd_epos.MoveToPosition(2, 360, false, true);
    pos_actual = abs(mcd_epos.GetPosition(2) % 360);
    ROS_INFO("Pos_actual = %d", pos_actual);
    pos_old = abs(mcd_epos.GetPosition(2));*/

/*
    while(1)
    {
	static bool tripode_1_cycle = true;
	static bool tripode_1_order = false;
	static bool tripode_2_cycle = false;
	static bool tripode_2_order = false;

	if((tripode_1_cycle == true) && (tripode_2_order == false))
	{
	    ROS_INFO("Muevo tripode 2");
	    mcd_epos.SetPositionProfile(2, 400, 12000, 12000);
	    //mcd_epos.MoveToPosition(3, -360, false, true);
	    mcd_epos.SetPositionProfile(6, 400, 12000, 12000);
	    mcd_epos.SetPositionProfile(1, 200, 12000, 12000);
	    mcd_epos.SetPositionProfile(5, 200, 12000, 12000);
	    mcd_epos.MoveToPosition(2, 60, false, true);
	    mcd_epos.MoveToPosition(6, 60, false, true);
	    mcd_epos.MoveToPosition(1, 60, false, true);
	    mcd_epos.MoveToPosition(5, 60, false, true);
	    mcd_epos.SetPositionProfile(2, 750, 12000, 12000);
	    mcd_epos.SetPositionProfile(6, 750, 12000, 12000);
	    mcd_epos.MoveToPosition(2, 300, false, false);
	    mcd_epos.MoveToPosition(6, 300, false, false);
	    tripode_2_cycle = false;
	    tripode_2_order = true;
	    tripode_2_estado_actual = suelo;
	}

	tripode_2_pos_actual = mcd_epos.GetPosition(2);

	if((tripode_2_pos_actual >= (tripode_2_pos_old + 360)) && (tripode_2_cycle == false))
	{
	    ROS_INFO("Paro tripode 2");
	    tripode_2_cycle = true;
	    //tripode_2_order = false;
	    tripode_2_pos_old = tripode_2_pos_actual;
	    tripode_1_order = false;
	    tripode_2_estado_actual = despegue;
	}

	if(tripode_2_cycle && !tripode_1_order)
	{
	    ROS_INFO("Muevo tripode 1");
//	    mcd_epos.MoveToPosition(1,  360, false, true);
	    //mcd_epos.MoveToPosition(4, -360, false, true);
//	    mcd_epos.MoveToPosition(5,  360, false, true);
	    tripode_1_order = true;
	    tripode_1_cycle = false;
	}

	//tripode_1_pos_actual = mcd_epos.GetPosition(1);
	//ROS_INFO("T1 posicion = %d", tripode_1_pos_actual);
*/
	/*if((tripode_1_pos_actual >= (tripode_1_pos_old + 360)) && (tripode_1_cycle == false))//(abs(tripode_1_pos_old) - 360) - 1)) && (tripode_1_cycle == false))
	{
	    ROS_INFO("Paro tripode 1");
	    tripode_1_cycle = true;
	    //tripode_1_order = false;
	    tripode_1_pos_old = tripode_1_pos_actual;
	    tripode_2_order = false;
	}*/
/*	//if(pos_actual == pos_old)
	//{
            mcd_epos.MoveToPosition(2, 360, false, true);
            mcd_epos.MoveToPosition(3, -360, false, true);
            mcd_epos.MoveToPosition(6, 360, false, true);
	//}
*/	

    //}
    /*int pata_4_consigna = 390;
    int pata_2_consigna = 330;
    int pata_6_consigna = 330;
    bool pata_2_flag = false;
    bool pata_6_flag = false;
    bool pata_4_flag = false;
    int pata_2_actual_pos = mcd_epos.GetPosition(2);
    int pata_6_actual_pos = mcd_epos.GetPosition(6);
    int pata_4_actual_pos = mcd_epos.GetPosition(4);
    int pata_2_old_pos = -999999;
    int pata_6_old_pos = -999999;
    int pata_4_old_pos = -999999;*/

    /**** Patas en posicion de setup ****/
    /*while((!pata_2_flag) && (!pata_6_flag) && (!pata_4_flag))
    {
        pata_2_actual_pos = mcd_epos.GetPosition(2);
        pata_6_actual_pos = mcd_epos.GetPosition(6);
        pata_4_actual_pos = mcd_epos.GetPosition(4);

        if(pata_2_actual_pos == pata_2_consigna)
            pata_2_flag = true;
        else
           mcd_epos.MoveToPosition(2, pata_2_consigna, true, true);

        if(pata_6_actual_pos == pata_6_consigna)
            pata_6_flag = true;
        else
           mcd_epos.MoveToPosition(6, pata_6_consigna, true, true);

        if(pata_4_actual_pos == pata_4_consigna)
            pata_4_flag = true;
        else
           mcd_epos.MoveToPosition(4, pata_4_consigna, true, true);
    */
        /*if(pata_2_old_pos != pata_2_actual_pos)
        {
            ROS_INFO("La posicion del motor 2 es: %d", pata_2_actual_pos);
            pata_2_old_pos = pata_2_actual_pos;
        }

        if(pata_6_old_pos != pata_6_actual_pos)
        {
            ROS_INFO("La posicion del motor 6 es: %d", pata_6_actual_pos);
            pata_6_old_pos = pata_6_actual_pos;
        }

        if(pata_4_old_pos != pata_4_actual_pos)
        {
            ROS_INFO("La posicion del motor 4 es: %d", pata_4_actual_pos);
            pata_4_old_pos = pata_4_actual_pos;
        }*/
    //}

    /*int vel_aire = 800;
    int vel_suelo = 945;

    bool fase_1 = false;    //Variable para indicar que estoy o que toca la fase 1. TRUE: La fase ha concluido con éxito
    bool fase_2 = true;    //Variable para indicar que estoy o que toca la fase 2. FALSE: Toca iniciar la fase
    bool pata_2_posicion = false;
    bool pata_4_posicion = false;
    bool pata_2_consigna_enviada = false;   //Variable para enviar una única vez la orden de movimiento
    bool pata_4_consigna_enviada = false;

    int ang_despegue = 30;
    int ang_aterrizaje = 330;

    int recorrido_vuelo = 300;
    int recorrido_suelo = 60;

    pata_2_old_pos = mcd_epos.GetPosition(2);
    pata_4_old_pos = mcd_epos.GetPosition(4);

    std::vector<int>  tripode_2 = {2, 3, 6};
    std::vector<bool> tripode_2_posicion_vector = {false, false}; // Vector que almacena que cada pata ha llegado a su consigna
    bool tripode_2_consigna_enviada = false; // Variable que indica que se ha enviado la consigna de posicion a todas las patas del tripode
    std::vector<bool> tripode_2_consigna_enviada_vector = {false, false}; // Vector que almacena que se ha enviado la consigna de posicion a cada pata del tripode. Cuando es todo TRUE se activa tripode_2_consigna_enviada
    std::vector<int>  tripode_2_pos_actual = {mcd_epos.GetPosition(tripode_2.at(0)), mcd_epos.GetPosition(tripode_2.at(1))};
    std::vector<int>  tripode_2_pos_old = {-999, -999};
    std::vector<bool> tripode_2_consigna_alcanzada_vector = {false, false}; // Vector que almacena que cada pata ha alcanzado su consigna
    bool tripode_2_consigna_alcanzada = false; // Variable que almacena que todas las patas han alcanzado su consigna

    ROS_INFO("Pata 2: %d -- Pata 4: %d", pata_2_actual_pos, pata_4_actual_pos);
    */
    //while(ros::ok())
    {
        //break;

    } // while(ros::ok())


    return 0;
    //device.close();
}
