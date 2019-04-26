#ifndef MOVEMENT_ORDERS_H
#define MOVEMENT_ORDERS_H

#include <epos_functions/epos_functions.h>

class movement_orders : public epos_functions
{
public:
    //epos_functions epos_functions_library;

    movement_orders();

    bool forward();
    bool left();
    bool right();
    /***********
     * stop order:
     * llama al metodo epos_functions::halt_movement(int)
     * como parametro hay que enviar el numero de motor (1:6)
     * *********/
    bool stop(int);

};

#endif // MOVEMENT_ORDERS_H
