#include "movement_orders/movement_orders.h"

movement_orders::movement_orders()
{
    //epos_functions epos_functions_library; // = epos_functions();
}

// Funcion para que el robot de 2 pasos con el tripode alterno.
// Primero da 1 paso con el T1 realizando la fase terrestre
// Segundo da 1 paso con el T2 realizando la fase terrestre
bool movement_orders::forward()
{
    std::vector<int>  patas = {1, 2, 3, 4, 5, 6};   // Vector que indica el numero de pata
    std::vector<int>  tripode_1 = {1, 4, 5};        // Vector que almacena los numeros de las patas del T1
    std::vector<int>  tripode_2 = {2, 3, 6};        // Vector que almacena los numeros de las patas del T2
    std::vector<bool> tripode_1_posicion_vector = {false, false, false};    // Vector que almacena que cada pata de T1 ha llegado a su consigna
    std::vector<bool> tripode_2_posicion_vector = {false, false, false};    // Vector que almacena que cada pata de T2 ha llegado a su consigna
    bool tripode_1_consigna_enviada = false;    // Variable que indica que se ha enviado la consigna de posicion a todas las patas de T1
    bool tripode_2_consigna_enviada = false;    // Variable que indica que se ha enviado la consigna de posicion a todas las patas de T2
    std::vector<bool> tripode_1_consigna_enviada_vector = {false, false, false};    // Vector que almacena que se ha enviado la consigna de posicion a cada pata de T1. Cuando es todo TRUE se activa tripode_1_consigna_enviada
    std::vector<bool> tripode_2_consigna_enviada_vector = {false, false, false};    // Vector que almacena que se ha enviado la consigna de posicion a cada pata de T2. Cuando es todo TRUE se activa tripode_2_consigna_enviada
    std::vector<int>  tripode_1_pos_actual = {epos_functions.GetPosition(tripode_1.at(0)), abs(epos_functions.GetPosition(tripode_1.at(1))), epos_functions.GetPosition(tripode_1.at(2))};
    std::vector<int>  tripode_2_pos_actual = {epos_functions.GetPosition(tripode_2.at(0)), abs(epos_functions.GetPosition(tripode_2.at(1))), epos_functions.GetPosition(tripode_2.at(2))};
    std::vector<int>  tripode_1_pos_old =  {999, -999, 999};    // Vector que inicializa la posicion de las patas de T1
    std::vector<int>  tripode_2_pos_old = {-999, 999, -999};    // Vector que inicializa la posicion de las patas de T2
    std::vector<bool> tripode_1_consigna_alcanzada_vector = {false, false, false};  // Vector que almacena que cada pata de T1 ha alcanzado su consigna
    std::vector<bool> tripode_2_consigna_alcanzada_vector = {false, false, false};  // Vector que almacena que cada pata de T2 ha alcanzado su consigna
    bool tripode_1_consigna_alcanzada = false;  // Variable que almacena que todas las patas de T1 han alcanzado su consigna
    bool tripode_2_consigna_alcanzada = false;  // Variable que almacena que todas las patas de T2 han alcanzado su consigna

    bool fase_1 = false;    //Variable para indicar que estoy o que toca la fase 1. TRUE: La fase ha concluido con éxito
    bool fase_2 = true;    //Variable para indicar que estoy o que toca la fase 2. FALSE: Toca iniciar la fase

    const int tripode_1_desfase = 260 - 1;
    const int tripode_2_desfase = 260 - 1;

    const int recorrido_vuelo = 300;
    const int recorrido_suelo = 60;

    // Inicio con la fase_1 = FALSE y la fase_2 = TRUE para forzar a que sea la fase_1 la primera
    /********************
     * Primera fase
     * T1 -> vel_suelo
     * T2 -> vel_aire
     * ******************/
    while(!fase_1 && fase_2)
    {
        // Enviar consigna para modo de funcionamiento y parametros a T2
        // Bucle for para recorrer las patas del tripode 2
        if(!tripode_2_consigna_enviada)
        {
            for(int i = 0; i < tripode_2.size(); i++)
            {
                // Envio la consigna de posicion y movimiento una única vez
                if(!tripode_2_consigna_enviada_vector.at(i))
                {
                    // Activo perfiles de velocidad de la fase
                    epos_functions.SetPositionProfile(tripode_2.at(i), vel_suelo, 12000, 12000);
                    if(i == 1)
                        epos_functions.MoveToPosition(tripode_2.at(i), -recorrido_suelo, false, true);
                    else
                        epos_functions.MoveToPosition(tripode_2.at(i), recorrido_suelo, false, true);
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
                    epos_functions.SetPositionProfile(tripode_1.at(i), vel_aire, 12000, 12000);
                    if((i % 2) == 0){
                        epos_functions.MoveToPosition(tripode_1.at(i), -recorrido_vuelo, false, true);
                    }else
                        epos_functions.MoveToPosition(tripode_1.at(i), recorrido_vuelo, false, true);
                    tripode_1_consigna_enviada_vector.at(i) = true;
                    //pata_2_consigna_enviada = true;
                }
            }
            //compruebo que se ha enviado la consigna a todas las patas
            tripode_1_consigna_enviada = std::all_of(tripode_1_consigna_enviada_vector.begin(), tripode_1_consigna_enviada_vector.end(), [](bool tripode_1_posicion_enviada) {return tripode_1_posicion_enviada;});
        }

        /**** Comprobación de las patas ****/
        for(int i = 0; i < tripode_2.size(); i++)
        {
            tripode_2_pos_actual.at(i) = (abs(epos_functions.GetPosition(tripode_2.at(i))) - tripode_2_desfase) % 360;
            //ROS_INFO("Posicion de la pata %d = %d -- Desfase = %d", tripode_2.at(i), tripode_2_pos_actual.at(i), epos_functions.GetPosition(tripode_2.at(i)));
        }
        for(int i = 0; i < tripode_1.size(); i++)
        {
            tripode_1_pos_actual.at(i) = (abs(epos_functions.GetPosition(tripode_1.at(i))) - tripode_1_desfase) % 360;
            //ROS_INFO("Posicion de la pata %d = %d -- Desfase = %d", tripode_1.at(i), tripode_1_pos_actual.at(i), epos_functions.GetPosition(tripode_1.at(i)));
        }

        // PROBLEMA: en el primer bucle empieza en 330º
        if(!tripode_2_consigna_alcanzada)
        {
            for(int i = 0; i < tripode_2.size(); i++)
            {
                if((tripode_2_pos_actual.at(i) >= (recorrido_suelo / 2)) && (tripode_2_pos_actual.at(i) < 300) && (!tripode_2_posicion_vector.at(i)))//(pata_2_old_pos + recorrido_suelo))
                {
                    epos_functions.HaltPositionMovement(tripode_2.at(i));
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
                    epos_functions.HaltPositionMovement(tripode_1.at(i));
                    ROS_INFO("La pata %d ha llegado a la posicion: %d", tripode_1.at(i), tripode_1_pos_actual.at(i));
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
        }
    } // while(!fase_1 && fase_2)

    // Inicio con la fase_2 = FALSE y la fase_1 = TRUE
    // Se hace este bucle para forzar a que sea la fase_1 la primera la que inicie el siguiente movimiento en el punto de aterrizaje
    /* Primera fase
     * T1 -> vel_aire
     * T2 -> vel_suelo
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
                    epos_functions.SetPositionProfile(tripode_2.at(i), vel_aire, 12000, 12000);
                    if(i == 1)
                        epos_functions.MoveToPosition(tripode_2.at(i), -recorrido_vuelo, false, true);
                    else
                        epos_functions.MoveToPosition(tripode_2.at(i), recorrido_vuelo, false, true);
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
                    epos_functions.SetPositionProfile(tripode_1.at(i), vel_suelo, 12000, 12000);
                    if((i % 2) == 0)
                        epos_functions.MoveToPosition(tripode_1.at(i), -recorrido_suelo, false, true);
                    else
                        epos_functions.MoveToPosition(tripode_1.at(i), recorrido_suelo, false, true);

                    tripode_1_consigna_enviada_vector.at(i) = true;
                    //pata_2_consigna_enviada = true;
                }
            }
            //compruebo que se ha enviado la consigna a todas las patas
            tripode_1_consigna_enviada = std::all_of(tripode_1_consigna_enviada_vector.begin(), tripode_1_consigna_enviada_vector.end(), [](bool tripode_1_posicion_enviada) {return tripode_1_posicion_enviada;});
        }

        // Compruebo que la pata ha llegado
        for(int i = 0; i < tripode_2.size(); i++)
        {
            tripode_2_pos_actual.at(i) = (abs(epos_functions.GetPosition(tripode_2.at(i))) - tripode_2_desfase) % 360;
            ROS_INFO("Posicion de la pata %d = %d -- Desfase = %d", tripode_2.at(i), tripode_2_pos_actual.at(i), epos_functions.GetPosition(tripode_2.at(i)));
        }
        for(int i = 0; i < tripode_1.size(); i++)
        {
            tripode_1_pos_actual.at(i) = (abs(epos_functions.GetPosition(tripode_1.at(i))) - tripode_1_desfase) % 360;
            ROS_INFO("Posicion de la pata %d = %d -- Desfase = %d", tripode_1.at(i), tripode_1_pos_actual.at(i), epos_functions.GetPosition(tripode_1.at(i)));
        }

        if(!tripode_2_consigna_alcanzada)
        {
            for(int i = 0; i < tripode_2.size(); i++)
            {
                if((tripode_2_pos_actual.at(i) >= (recorrido_vuelo + 30)) && (!tripode_2_posicion_vector.at(i)))//(pata_2_old_pos + recorrido_suelo))
                {
                    epos_functions.HaltPositionMovement(tripode_2.at(i));
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
                    epos_functions.HaltPositionMovement(tripode_1.at(i));
                    ROS_INFO("La pata %d ha llegado a la posicion: %d", tripode_1.at(i), tripode_1_pos_actual.at(i));
                    tripode_1_pos_old.at(i) = tripode_1_pos_actual.at(i);
                    //pata_2_old_pos = pata_2_actual_pos % 360;
                    tripode_1_consigna_alcanzada_vector.at(i) = true;
                    //pata_2_posicion = true;
                }
            }
            tripode_1_consigna_alcanzada = std::all_of(tripode_1_consigna_alcanzada_vector.begin(), tripode_1_consigna_alcanzada_vector.end(), [](bool tripode_1_consigna_alcanzada_flag) {return tripode_1_consigna_alcanzada_flag;});
        }

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

            return true;
        }
    } // while(fase_1 && !fase_2)

}

bool movement_orders::stop(int motor)
{
    //bool flag = epos_functions_library::HaltMovementOrder(motor);
    return motor;

}
