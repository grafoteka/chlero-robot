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
    // Activo los perfiles de posicion y les doy un perfil de velocidad
    for(int i = 1; i <= 6; i++)
    {
        mcd_epos.ActivateProfilePosition(i);
        mcd_epos.SetPositionProfile(i, 800, 12000, 12000);
    }

    mcd_epos.SetPositionProfile(4, 945.45, 12000, 12000);

    //result = mcd_epos.MoveToPosition(6, 132000, false, true);

    int pata_4_consigna = 390;
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
    int pata_4_old_pos = -999999;

    /**** Patas en posicion de setup ****/
    while((!pata_2_flag) && (!pata_6_flag) && (!pata_4_flag))
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
    }

    int vel_aire = 800;
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

    std::vector<int> tripode_2 = {2, 6};
    std::vector<bool> tripode_2_posicion = {false, false};
    std::vector<bool> tripode_2_consigna_enviada = {false, false};

    ROS_INFO("Pata 2: %d -- Pata 4: %d", pata_2_actual_pos, pata_4_actual_pos);

    while(ros::ok())
    {
        //break;
        /**** tripode alterno ****/
        // Inicio con la fase_1 = FALSE y la fase_2 = TRUE para forzar a que sea la fase_1 la primera
        /* Primera fase
         * T1 -> vel_suelo
         * T2 -> vel_aire
        */
        while(!fase_1 && fase_2)
        {
            //ROS_INFO("FASE 1");

            // Envio la consigna de posicion y movimiento una única vez
            if(!pata_2_consigna_enviada)
            {
                // Activo perfiles de velocidad de la fase
                mcd_epos.SetPositionProfile(2, 800, 12000, 12000);
                mcd_epos.MoveToPosition(2, recorrido_suelo, false, true);
                pata_2_consigna_enviada = true;
            }
            if(!pata_4_consigna_enviada)
            {
                // Activo perfiles de velocidad de la fase
                mcd_epos.SetPositionProfile(4, 945, 12000, 12000);
                mcd_epos.MoveToPosition(4, recorrido_vuelo, false, true);
                pata_4_consigna_enviada = true;
            }

            // Compruebo que la pata ha llegado
            pata_2_actual_pos = mcd_epos.GetPosition(2) % 360;
            pata_4_actual_pos = mcd_epos.GetPosition(4) % 360;
            //ROS_INFO("Pata 2 = %d  -- Old 2 = %d --  Pata 4 = %d -- Old 4 = %d", pata_2_actual_pos, pata_2_old_pos, pata_4_actual_pos, pata_4_old_pos);

            // La pata 2 debe de llegar a 30º -> Recorre 60º (recorrido_suelo)
            // PROBLEMA: en el primer bucle empieza en 330º
            if((pata_2_actual_pos >= (recorrido_suelo / 2)) && (pata_2_actual_pos < 300) && (!pata_2_posicion))//(pata_2_old_pos + recorrido_suelo))
            {
                mcd_epos.HaltPositionMovement(2);
                ROS_INFO("La pata 2 ha llegado a la posicion: %d", pata_2_actual_pos);
                pata_2_old_pos = pata_2_actual_pos % 360;
                pata_2_posicion = true;
            }

            // La pata 4 debe de llegar a 330º
            if((pata_4_actual_pos >= (recorrido_vuelo + 30)) && (!pata_4_posicion))//(pata_4_old_pos + recorrido_vuelo))
            {
                mcd_epos.HaltPositionMovement(4);
                ROS_INFO("La pata 4 ha llegado a la posicion: %d", (pata_4_actual_pos % 360));
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
        } // while(!fase_1 && fase_2)


        /* Segunda fase
         * Pata 2 -> vel aire
         * Pata 4 -> vel suelo
        */
        while(fase_1 && !fase_2)
        {
            // Envio la consigna de posicion y movimiento una única vez
            if(!pata_2_consigna_enviada)
            {
                ROS_INFO("FASE 2");
                // Correción para el posible desfase que acumulan las patas
                //r_vuelo = 300º
                //La pata debe empezar en 30º (390 % 360) -> 300º grados de vuelo, pero empieza con un error
                // recorrido_vuelo = recorrido_vuelo - pata_2_actual_pos;
                // En lugar de actual_pos uso old_pos porque está actualizada y con el módulo
                //recorrido_vuelo = recorrido_vuelo - pata_2_old_pos;
                //ROS_INFO("RECORRIDO PATA 2 = %d", recorrido_vuelo);
                // Activo perfiles de velocidad de la fase
                mcd_epos.SetPositionProfile(2, 945, 12000, 12000);
                mcd_epos.MoveToPosition(2, recorrido_vuelo, false, true);
                pata_2_consigna_enviada = true;
            }
            if(!pata_4_consigna_enviada)
            {
                // Activo perfiles de velocidad de la fase
                mcd_epos.SetPositionProfile(4, 800, 12000, 12000);
                mcd_epos.MoveToPosition(4, recorrido_suelo, false, true);
                pata_4_consigna_enviada = true;
            }

            // Compruebo que la pata ha llegado
            pata_2_actual_pos = mcd_epos.GetPosition(2) % 360;
            pata_4_actual_pos = mcd_epos.GetPosition(4) % 360;
            //ROS_INFO("Pata 2 = %d  --  Pata 4 = %d", pata_2_actual_pos % 360, pata_4_actual_pos % 360);
            //ROS_INFO("Pata 2 = %d  -- Old 2 = %d --  Pata 4 = %d -- Old 4 = %d", pata_2_actual_pos, pata_2_old_pos, pata_4_actual_pos, pata_4_old_pos);
            if((pata_2_actual_pos) >= (pata_2_old_pos + recorrido_vuelo))
            {
                mcd_epos.HaltPositionMovement(2);
                ROS_INFO("La pata 2 ha llegado a la posicion: %d", pata_2_actual_pos % 360);
                pata_2_old_pos = pata_2_actual_pos;
                pata_2_posicion = true;
            }
            //if(pata_4_actual_pos >= (pata_4_old_pos + recorrido_vuelo))
            if((pata_4_actual_pos >= (recorrido_suelo / 2)) && (pata_4_actual_pos < 300) && (!pata_4_posicion))//(pata_2_old_pos + recorrido_suelo))
            {
                mcd_epos.HaltPositionMovement(4);
                ROS_INFO("La pata 4 ha llegado a la posicion: %d", (pata_4_actual_pos % 360));
                pata_4_old_pos = pata_4_actual_pos;
                pata_4_posicion = true;
            }

            if(pata_2_posicion && pata_4_posicion)
            {
                fase_1 = false;
                fase_2 = true;
                pata_2_posicion = false;
                pata_4_posicion = false;
                pata_2_consigna_enviada = false;
                pata_4_consigna_enviada = false;
                ROS_INFO("Fin de fase 2");
            }
        } // while(fase_1 && !fase_2)


    } // while(ros::ok())


    return 0;
    //device.close();
}
