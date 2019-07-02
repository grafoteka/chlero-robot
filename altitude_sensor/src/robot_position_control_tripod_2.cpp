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
    /*for(int i = 1; i <= 6; i++)
    {
        mcd_epos.ActivateProfilePosition(i);
        mcd_epos.SetPositionProfile(i, 800, 12000, 12000);
    }

    mcd_epos.SetPositionProfile(4, 945.45, 12000, 12000);
    */
    //result = mcd_epos.MoveToPosition(6, 132000, false, true);

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
    int pata_4_old_pos = -999999;
    */
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

    int vel_aire = 1000;
    int vel_suelo = 250;

    const int tripode_1_desfase = 260 - 1;
    const int tripode_2_desfase = 260 - 1;

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

    int pata_2_old_pos =  999;
    int pata_4_old_pos = -999; // = mcd_epos.GetPosition(4);

    std::vector<int>  patas = {1, 2, 3, 4, 5, 6};
    std::vector<int>  tripode_1 = {1, 4, 5};
    std::vector<int>  tripode_2 = {2, 3, 6};
    std::vector<bool> tripode_1_posicion_vector = {false, false, false};
    std::vector<bool> tripode_2_posicion_vector = {false, false, false}; // Vector que almacena que cada pata ha llegado a su consigna
    bool tripode_1_consigna_enviada = false;
    bool tripode_2_consigna_enviada = false; // Variable que indica que se ha enviado la consigna de posicion a todas las patas del tripode
    std::vector<bool> tripode_1_consigna_enviada_vector = {false, false, false};
    std::vector<bool> tripode_2_consigna_enviada_vector = {false, false, false}; // Vector que almacena que se ha enviado la consigna de posicion a cada pata del tripode. Cuando es todo TRUE se activa tripode_2_consigna_enviada
    std::vector<int>  tripode_1_pos_actual = {mcd_epos.GetPosition(tripode_1.at(0)), abs(mcd_epos.GetPosition(tripode_1.at(1))), mcd_epos.GetPosition(tripode_1.at(2))};
    std::vector<int>  tripode_2_pos_actual = {mcd_epos.GetPosition(tripode_2.at(0)), abs(mcd_epos.GetPosition(tripode_2.at(1))), mcd_epos.GetPosition(tripode_2.at(2))};
    std::vector<int>  tripode_1_pos_old = {999, -999, 999};
    std::vector<int>  tripode_2_pos_old = {-999, 999, -999};
    std::vector<bool> tripode_1_consigna_alcanzada_vector = {false, false, false};
    std::vector<bool> tripode_2_consigna_alcanzada_vector = {false, false, false}; // Vector que almacena que cada pata ha alcanzado su consigna
    bool tripode_1_consigna_alcanzada = false;
    bool tripode_2_consigna_alcanzada = false; // Variable que almacena que todas las patas han alcanzado su consigna

    int pata_2_actual_pos = mcd_epos.GetPosition(2) - tripode_2_desfase;
    int pata_4_actual_pos = mcd_epos.GetPosition(4) - tripode_1_desfase;

    for(int i = 0; i < patas.size(); i++)
    {
        int posicion;
        if (i == 0)
            posicion = abs(mcd_epos.GetPosition(tripode_1.at(0))) - tripode_1_desfase;

        else if (i == 1)
            posicion = abs(mcd_epos.GetPosition(tripode_2.at(0))) - tripode_2_desfase;

        else if (i == 2)
            posicion = abs(mcd_epos.GetPosition(tripode_2.at(1))) - tripode_2_desfase;

        else if (i == 3)
            posicion = abs(mcd_epos.GetPosition(tripode_1.at(1))) - tripode_1_desfase;

        else if (i == 4)
            posicion = abs(mcd_epos.GetPosition(tripode_1.at(2))) - tripode_1_desfase;

        else if (i == 5)
            posicion = abs(mcd_epos.GetPosition(tripode_2.at(2))) - tripode_2_desfase;

        ROS_INFO("Pata %d -- Posicion %d", patas.at(i), posicion);
    }

    /*}


   ROS_INFO("Pata 4: %d", pata_4_actual_pos);
*/
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

            /*if(!pata_2_consigna_enviada)
            {
                // Activo perfiles de velocidad de la fase
                mcd_epos.SetPositionProfile(2, 250, 12000, 12000);
                mcd_epos.MoveToPosition(2, recorrido_suelo, false, true);
                pata_2_consigna_enviada = true;
            }*/
            // Bucle for para recorrer las patas del tripode 2
            if(!tripode_2_consigna_enviada)
            {
                for(int i = 0; i < tripode_2.size(); i++)
                {
                    // Envio la consigna de posicion y movimiento una única vez
                    if(!tripode_2_consigna_enviada_vector.at(i))
                    {
                        // Activo perfiles de velocidad de la fase
                        mcd_epos.SetPositionProfile(tripode_2.at(i), vel_suelo, 12000, 12000);
                        if(i == 1)
                            mcd_epos.MoveToPosition(tripode_2.at(i), -recorrido_suelo, false, true);
                        else
                            mcd_epos.MoveToPosition(tripode_2.at(i), recorrido_suelo, false, true);
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
                        mcd_epos.SetPositionProfile(tripode_1.at(i), vel_aire, 12000, 12000);
                        if((i % 2) == 0){
                            mcd_epos.MoveToPosition(tripode_1.at(i), -recorrido_vuelo, false, true);
                        }else
                            mcd_epos.MoveToPosition(tripode_1.at(i), recorrido_vuelo, false, true);
                        tripode_1_consigna_enviada_vector.at(i) = true;
                        //pata_2_consigna_enviada = true;
                    }
                }
                //compruebo que se ha enviado la consigna a todas las patas
                tripode_1_consigna_enviada = std::all_of(tripode_1_consigna_enviada_vector.begin(), tripode_1_consigna_enviada_vector.end(), [](bool tripode_1_posicion_enviada) {return tripode_1_posicion_enviada;});
            }

            /*if(!pata_4_consigna_enviada)
            {
                // Activo perfiles de velocidad de la fase
                mcd_epos.SetPositionProfile(4, 300, 12000, 12000);
                mcd_epos.MoveToPosition(4, recorrido_vuelo, false, true);
                pata_4_consigna_enviada = true;
            }*/

            /**** Comprobación de las patas ****/
            // Lectura de la posición actual de las patas
            //pata_2_actual_pos = abs((mcd_epos.GetPosition(2) - tripode_2_desfase) % 360);
            //ROS_INFO("Posicion de la pata 2 = %d -- Desfase = %d", pata_2_actual_pos, mcd_epos.GetPosition(2));
            for(int i = 0; i < tripode_2.size(); i++)
            {
                tripode_2_pos_actual.at(i) = (abs(mcd_epos.GetPosition(tripode_2.at(i))) - tripode_2_desfase) % 360;
                //ROS_INFO("Posicion de la pata %d = %d -- Desfase = %d", tripode_2.at(i), tripode_2_pos_actual.at(i), mcd_epos.GetPosition(tripode_2.at(i)));
            }
            for(int i = 0; i < tripode_1.size(); i++)
            {
                tripode_1_pos_actual.at(i) = (abs(mcd_epos.GetPosition(tripode_1.at(i))) - tripode_1_desfase) % 360;
                //ROS_INFO("Posicion de la pata %d = %d -- Desfase = %d", tripode_1.at(i), tripode_1_pos_actual.at(i), mcd_epos.GetPosition(tripode_1.at(i)));
            }
            //
            //if(pata_4_old_pos != pata_4_actual_pos)
            //{

            //    pata_4_old_pos = pata_4_actual_pos;
            //}
            //ROS_INFO("Pata 2 = %d  -- Old 2 = %d --  Pata 4 = %d -- Old 4 = %d", pata_2_actual_pos, pata_2_old_pos, pata_4_actual_pos, pata_4_old_pos);

            // La pata 2 debe de llegar a 30º -> Recorre 60º (recorrido_suelo)
            /*if((abs(pata_2_actual_pos) >= (recorrido_suelo / 2)) && (pata_2_actual_pos < 300) && (!pata_2_posicion))//(pata_4_old_pos + recorrido_vuelo))
            {
                mcd_epos.HaltPositionMovement(2);
                ROS_INFO("La pata 2 ha llegado a la posicion: %d", (pata_2_actual_pos % 360));
                pata_2_old_pos = pata_2_actual_pos;
                pata_2_posicion = true;
            }*/
            // PROBLEMA: en el primer bucle empieza en 330º
            if(!tripode_2_consigna_alcanzada)
            {
                for(int i = 0; i < tripode_2.size(); i++)
                {
                    if((tripode_2_pos_actual.at(i) >= (recorrido_suelo / 2)) && (tripode_2_pos_actual.at(i) < 300) && (!tripode_2_posicion_vector.at(i)))//(pata_2_old_pos + recorrido_suelo))
                    {
                        mcd_epos.HaltPositionMovement(tripode_2.at(i));
                        ROS_INFO("La pata %d ha llegado a la posicion: %d", tripode_2.at(i), tripode_2_pos_actual.at(i));
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
                    if((abs(tripode_1_pos_actual.at(i)) >= (recorrido_vuelo + 30)) && (!tripode_1_posicion_vector.at(i)))//(pata_2_old_pos + recorrido_suelo))
                    {
                        mcd_epos.HaltPositionMovement(tripode_1.at(i));
                        ROS_INFO("La pata %d ha llegado a la posicion: %d", tripode_1.at(i), tripode_1_pos_actual.at(i));
                        tripode_1_pos_old.at(i) = tripode_1_pos_actual.at(i);
                        //pata_2_old_pos = pata_2_actual_pos % 360;
                        tripode_1_consigna_alcanzada_vector.at(i) = true;
                        //pata_2_posicion = true;
                    }
                }
                tripode_1_consigna_alcanzada = std::all_of(tripode_1_consigna_alcanzada_vector.begin(), tripode_1_consigna_alcanzada_vector.end(), [](bool tripode_1_consigna_alcanzada_flag) {return tripode_1_consigna_alcanzada_flag;});
            }

            // La pata 4 debe de llegar a 330º
            /*if((abs(pata_4_actual_pos) >= (recorrido_vuelo + 30)) && (!pata_4_posicion))//(pata_4_old_pos + recorrido_vuelo))
            {
                mcd_epos.HaltPositionMovement(4);
                ROS_INFO("La pata 4 ha llegado a la posicion: %d", (pata_4_actual_pos % 360));
                pata_4_old_pos = pata_4_actual_pos;
                pata_4_posicion = true;
            }*/

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

                pata_2_posicion = false;
                pata_4_posicion = false;
                pata_2_consigna_enviada = false;
                pata_4_consigna_enviada = false;
            }
        } // while(!fase_1 && fase_2)


        //break;

        /* Segunda fase
         * Pata 2 -> vel aire
         * Pata 4 -> vel suelo
        */
        while(fase_1 && !fase_2)
        {
            // Envio la consigna de posicion y movimiento una única vez
            // Bucle for para recorrer las patas del tripode 2
            if(!tripode_2_consigna_enviada)
            {
                for(int i = 0; i < tripode_2.size(); i++)
                {
                    // Envio la consigna de posicion y movimiento una única vez
                    if(!tripode_2_consigna_enviada_vector.at(i))
                    {
                        // Activo perfiles de velocidad de la fase
                        mcd_epos.SetPositionProfile(tripode_2.at(i), vel_aire, 12000, 12000);
                        if(i == 1)
                            mcd_epos.MoveToPosition(tripode_2.at(i), -recorrido_vuelo, false, true);
                        else
                            mcd_epos.MoveToPosition(tripode_2.at(i), recorrido_vuelo, false, true);
                        tripode_2_consigna_enviada_vector.at(i) = true;
                        //pata_2_consigna_enviada = true;
                    }
                }
                //compruebo que se ha enviado la consigna a todas las patas
                tripode_2_consigna_enviada = std::all_of(tripode_2_consigna_enviada_vector.begin(), tripode_2_consigna_enviada_vector.end(), [](bool tripode_2_posicion_enviada) {return tripode_2_posicion_enviada;});
            }

            if(!tripode_1_consigna_enviada)
            {
                for(int i = 0; i < tripode_1.size(); i++)
                {
                    // Envio la consigna de posicion y movimiento una única vez
                    if(!tripode_1_consigna_enviada_vector.at(i))
                    {
                        // Activo perfiles de velocidad de la fase
                        mcd_epos.SetPositionProfile(tripode_1.at(i), vel_suelo, 12000, 12000);
                        if((i % 2) == 0)
                            mcd_epos.MoveToPosition(tripode_1.at(i), -recorrido_suelo, false, true);
                        else
                            mcd_epos.MoveToPosition(tripode_1.at(i), recorrido_suelo, false, true);

                        tripode_1_consigna_enviada_vector.at(i) = true;
                        //pata_2_consigna_enviada = true;
                    }
                }
                //compruebo que se ha enviado la consigna a todas las patas
                tripode_1_consigna_enviada = std::all_of(tripode_1_consigna_enviada_vector.begin(), tripode_1_consigna_enviada_vector.end(), [](bool tripode_1_posicion_enviada) {return tripode_1_posicion_enviada;});
            }

            /*if(!pata_4_consigna_enviada)
            {
                // Activo perfiles de velocidad de la fase
                mcd_epos.SetPositionProfile(4, 250, 12000, 12000);
                mcd_epos.MoveToPosition(4, recorrido_suelo, false, true);
                pata_4_consigna_enviada = true;
            }*/

            // Compruebo que la pata ha llegado
            for(int i = 0; i < tripode_2.size(); i++)
            {
                tripode_2_pos_actual.at(i) = (abs(mcd_epos.GetPosition(tripode_2.at(i))) - tripode_2_desfase) % 360;
                ROS_INFO("Posicion de la pata %d = %d -- Desfase = %d", tripode_2.at(i), tripode_2_pos_actual.at(i), mcd_epos.GetPosition(tripode_2.at(i)));
            }
            for(int i = 0; i < tripode_1.size(); i++)
            {
                tripode_1_pos_actual.at(i) = (abs(mcd_epos.GetPosition(tripode_1.at(i))) - tripode_1_desfase) % 360;
                ROS_INFO("Posicion de la pata %d = %d -- Desfase = %d", tripode_1.at(i), tripode_1_pos_actual.at(i), mcd_epos.GetPosition(tripode_1.at(i)));
            }
            /*for(int i = 0; i < tripode_2.size(); i++)
            {
                tripode_2_pos_actual.at(i) = mcd_epos.GetPosition(tripode_2.at(i)) % 360;
            }
            pata_4_actual_pos = mcd_epos.GetPosition(4) % 360;*/
            //ROS_INFO("Pata 2 = %d  -- Old 2 = %d --  Pata 4 = %d -- Old 4 = %d", pata_2_actual_pos, pata_2_old_pos, pata_4_actual_pos, pata_4_old_pos);

            // La pata 2 debe de llegar a 30º -> Recorre 60º (recorrido_suelo)
            // PROBLEMA: en el primer bucle empieza en 330º
            if(!tripode_2_consigna_alcanzada)
            {
                for(int i = 0; i < tripode_2.size(); i++)
                {
                    if((tripode_2_pos_actual.at(i) >= (recorrido_vuelo + 30)) && (!tripode_2_posicion_vector.at(i)))//(pata_2_old_pos + recorrido_suelo))
                    {
                        mcd_epos.HaltPositionMovement(tripode_2.at(i));
                        ROS_INFO("La pata %d ha llegado a la posicion: %d", tripode_2.at(i), tripode_2_pos_actual.at(i));
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
                    if((abs(tripode_1_pos_actual.at(i)) >= (recorrido_suelo / 2)) && (tripode_1_pos_actual.at(i) < 300) && (!tripode_1_posicion_vector.at(i)))
                    {
                        mcd_epos.HaltPositionMovement(tripode_1.at(i));
                        ROS_INFO("La pata %d ha llegado a la posicion: %d", tripode_1.at(i), tripode_1_pos_actual.at(i));
                        tripode_1_pos_old.at(i) = tripode_1_pos_actual.at(i);
                        //pata_2_old_pos = pata_2_actual_pos % 360;
                        tripode_1_consigna_alcanzada_vector.at(i) = true;
                        //pata_2_posicion = true;
                    }
                }
                tripode_1_consigna_alcanzada = std::all_of(tripode_1_consigna_alcanzada_vector.begin(), tripode_1_consigna_alcanzada_vector.end(), [](bool tripode_1_consigna_alcanzada_flag) {return tripode_1_consigna_alcanzada_flag;});
            }
            /*if(!tripode_2_consigna_alcanzada)
            {
                for(int i = 0; i < tripode_2.size(); i++)
                {
                    if((tripode_2_pos_actual.at(i) >= (tripode_2_pos_old.at(i) + recorrido_vuelo)) && (!tripode_2_posicion_vector.at(i)))//(pata_2_old_pos + recorrido_suelo))
                    {
                        mcd_epos.HaltPositionMovement(tripode_2.at(i));
                        ROS_INFO("La pata %d ha llegado a la posicion: %d", tripode_2.at(i), tripode_2_pos_actual.at(i));
                        tripode_2_pos_old.at(i) = tripode_2_pos_actual.at(i);
                        //pata_2_old_pos = pata_2_actual_pos % 360;
                        tripode_2_consigna_alcanzada_vector.at(i) = true;
                        //pata_2_posicion = true;
                    }
                }
                tripode_2_consigna_alcanzada = std::all_of(tripode_2_consigna_alcanzada_vector.begin(), tripode_2_consigna_alcanzada_vector.end(), [](bool tripode_2_consigna_alcanzada_flag) {return tripode_2_consigna_alcanzada_flag;});
            }*/
            //pata_2_actual_pos = mcd_epos.GetPosition(2) % 360;
            //pata_4_actual_pos = mcd_epos.GetPosition(4) % 360;
            //ROS_INFO("Pata 2 = %d  --  Pata 4 = %d", pata_2_actual_pos % 360, pata_4_actual_pos % 360);
            //ROS_INFO("Pata 2 = %d  -- Old 2 = %d --  Pata 4 = %d -- Old 4 = %d", pata_2_actual_pos, pata_2_old_pos, pata_4_actual_pos, pata_4_old_pos);
            /*if((pata_2_actual_pos) >= (pata_2_old_pos + recorrido_vuelo))
            {
                mcd_epos.HaltPositionMovement(2);
                ROS_INFO("La pata 2 ha llegado a la posicion: %d", pata_2_actual_pos % 360);
                pata_2_old_pos = pata_2_actual_pos;
                pata_2_posicion = true;
            }*/
            //if(pata_4_actual_pos >= (pata_4_old_pos + recorrido_vuelo))
            /*if((pata_4_actual_pos >= (recorrido_suelo / 2)) && (pata_4_actual_pos < 300) && (!pata_4_posicion))//(pata_2_old_pos + recorrido_suelo))
            {
                mcd_epos.HaltPositionMovement(4);
                ROS_INFO("La pata 4 ha llegado a la posicion: %d", (pata_4_actual_pos % 360));
                pata_4_old_pos = pata_4_actual_pos;
                pata_4_posicion = true;
            }*/

            /**** Salida de la Fase 2 ****/
            if(tripode_1_consigna_alcanzada && tripode_2_consigna_alcanzada)// && tripode_1_consigna_alcanzada)
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

                pata_2_posicion = false;
                pata_4_posicion = false;
                pata_2_consigna_enviada = false;
                pata_4_consigna_enviada = false;
            }
        } // while(fase_1 && !fase_2)
        //break;

    } // while(ros::ok())


    return 0;
    //device.close();
}
