#include "epos_functions/epos_functions.h"

epos_functions::epos_functions()
{
    this->ulErrorCode = 0;
    this->keyHandle = 0;
    this->node_id = 0;
    this->deviceName = "EPOS";
    this->protocolStackName = "MAXON_RS232";
    this->interfaceName = "RS_232";
    this->portName = "/dev/ttyS1";
    this->baudrate = 115200;

    int lResult = MMC_FAILED;

    ROS_INFO("Entro en el constructor");

    // Iniciamos las Epos
    for(int motor = 1; motor <= 6; motor++)
    {
        this->node_id = motor;
        ROS_INFO("Iniciando el motor #%d", this->node_id);

        if((lResult = OpenDevice(&this->ulErrorCode, this->node_id)!=MMC_SUCCESS))
        {
                LogError("OpenDevice", lResult, ulErrorCode);
                //return lResult;
        }
        //lResult=this->OpenDevice(&this->ulErrorCode, this->node_id, this->keyHandle, this->deviceName, this->protocolStackName, this->interfaceName, this->portName, this->baudrate);

    }

    // Eliminamos los errores
    if((lResult = this->ClearErrors(&ulErrorCode))!=MMC_SUCCESS)
        {
            LogError("PrepareDemo", lResult, ulErrorCode);
            //return lResult;
        }
}

void epos_functions::LogInfo(std::string message)
{
    std::cout << message << std::endl;
}
void epos_functions::LogError(std::string functionName, int p_lResult, unsigned int p_ulErrorCode)
{
    std::cerr << "La funcion de Maxon: " << functionName << " failed (result=" << p_lResult << ", errorCode=0x" << std::hex << p_ulErrorCode << ")"<< std::endl;
}

/*
 * Esta funcion es la que elimina los errores en las EPOS cuando se inicializan la primera vez
 * Por lo normal el fallo siempre está en la EPOS 5.
 * La solución es eliminar el error
 */
int epos_functions::ClearErrors(unsigned int *p_pErrorCode)
{
    int lResult = MMC_SUCCESS;
    int oIsFault = 0;   // antes era bool
    for(unsigned short node_id = 1; node_id <= 6; node_id++)
    {
        if(VCS_GetFaultState(this->keyHandle, node_id, &oIsFault, p_pErrorCode ) == 0)
        {
            LogError("VCS_GetFaultState", lResult, *p_pErrorCode);
            lResult = MMC_FAILED;
        }

        if(lResult==0)
        {
            if(oIsFault)
            {
                //std::stringstream msg;
                //msg << "Errores eliminados del motor #" << node_id << std::endl;
                //LogInfo(msg.str());
                ROS_INFO("Errores eliminados del motor #%d", node_id);

                if(VCS_ClearFault(keyHandle, node_id, p_pErrorCode) == 0)
                {
                    LogError("VCS_ClearFault", lResult, *p_pErrorCode);
                    lResult = MMC_FAILED;
                }
            }

            if(lResult==0)
            {
                int oIsEnabled = 0; //Antes era bool pero no lo puede convertir a int.

                if(VCS_GetEnableState(keyHandle, node_id, &oIsEnabled, p_pErrorCode) == 0)
                {
                    LogError("VCS_GetEnableState", lResult, *p_pErrorCode);
                    lResult = MMC_FAILED;
                }

                if(lResult==0)
                {
                    if(!oIsEnabled)
                    {
                        if(VCS_SetEnableState(keyHandle, node_id, p_pErrorCode) == 0)
                        {
                            LogError("VCS_SetEnableState", lResult, *p_pErrorCode);
                            lResult = MMC_FAILED;
                        }
                    }
                }
            }
        }
    }
    return lResult;
}

/*int epos_functions::OpenDevice(unsigned int* errorCode, int node_id,
               void* pKeyHandle, std::string deviceName,
               std::string protocolStackName, std::string interfaceName,
               std::string portName, int baudrate)*/
int epos_functions::OpenDevice(unsigned int* errorCode, int node_id)
{
    int lResult = MMC_FAILED;
    char* char_deviceName = new char[255];
    char* char_protocolStackName = new char[255];
    char* char_interfaceName = new char[255];
    char* char_portName = new char[255];

    std::string device_name = "EPOS";
    std::string interface_name = "RS232";
    int baudrate_local = 115200;

    strcpy(char_deviceName, device_name.c_str());
    strcpy(char_protocolStackName, protocolStackName.c_str());
    strcpy(char_interfaceName, interface_name.c_str());
    strcpy(char_portName, portName.c_str());

    //LogInfo("Open device...");
    ROS_INFO("Open device #%d", node_id);

    this->keyHandle = VCS_OpenDevice(char_deviceName, char_protocolStackName, char_interfaceName, char_portName, errorCode);

    if(this->keyHandle!=0 && *errorCode == 0)
    {
        unsigned int lBaudrate = 0;
        unsigned int lTimeout = 0;

        if(VCS_GetProtocolStackSettings(this->keyHandle, &lBaudrate, &lTimeout, errorCode)!=0)
        {
            if(VCS_SetProtocolStackSettings(this->keyHandle, baudrate_local, lTimeout, errorCode)!=0)
            {
                if(VCS_GetProtocolStackSettings(this->keyHandle, &lBaudrate, &lTimeout, errorCode)!=0)
                {
                    if(baudrate == (int)lBaudrate)
                    {
                        lResult = MMC_SUCCESS;
                    }
                }
            }
        }
    }
    else
    {
        this->keyHandle = 0;
    }

    delete []char_deviceName;
    delete []char_protocolStackName;
    delete []char_interfaceName;
    delete []char_portName;

        if(lResult == MMC_SUCCESS)
        {
            //LogInfo("Device openned correctly");
            ROS_INFO("Device openned correctly", node_id);
        }

    return lResult;
}

/**** Funcion para activar el perfil en posicion ****/
int epos_functions::ActivateProfilePosition(int motor)
{
    void* keyHandle_local = keyHandle;
    unsigned int error_code = 0;
    if(int result = VCS_ActivateProfilePositionMode(keyHandle_local, motor, &error_code) == 0)
    {
        ROS_INFO("ERROR: No se ha podido activar el modo posicion en el motor %d", motor);
    }
}

/**** Funcion para definir el trapecio de velocidad en el perfil de posicion ****/
int epos_functions::SetPositionProfile(int motor, int velocity, int acceleration, int deceleration)
{
    void* keyHandle_local = keyHandle;
    unsigned int error_code = 0;
    int result = 0;
    if((result = VCS_SetPositionProfile(keyHandle_local, motor, velocity, acceleration, deceleration, &error_code)) == 0)
    {
        ROS_INFO("ERROR: No se ha podido cargar el perfil de velocidad en el motor %d", motor);
    }
    return result;
}

/**** Función para mover el motor hasta una posición con un perfil de velocidad ****/
int epos_functions::MoveToPosition(int motor, int position, bool absolute, bool inmediately)
{
    void* keyHandle_local = keyHandle;
    unsigned int error_code = 0;
    int result = 0;
    int position_encoder = position * (4000 * 33) / 360;    // Posicion_deseada * (encoder * reductora) / grados_una_vuelta
    if((result = VCS_MoveToPosition(keyHandle_local, motor, position_encoder, absolute, inmediately, &error_code)) == 0)
    {
        ROS_INFO("ERROR: No se ha podido llegar a la consigna en el motor %d", motor);
    }
    return result;
}

/**** Función para obtener la posición actual del motor ****/
int epos_functions::GetPosition(int motor)
{
    void* keyHandle_local = keyHandle;
    unsigned int error_code = 0;
    int result = 0;
    int position_actual_raw = 0;
    int position_actual = 0;
    if((result = VCS_GetPositionIs(keyHandle_local, motor, &position_actual_raw, &error_code)) == 0)
    {
        ROS_INFO("ERROR: No se ha podido llegar a la consigna en el motor %d", motor);
        result = 0;
        return result;
    }
    position_actual = (position_actual_raw * 360) / (4000 * 33); //Encoder de cuadratura de 4 pulsos, con un encoder de 1000 pulsos -> 4000 pulsos por vuelta.
    return position_actual;
}

/**** Función para detener el movimiento en posición del motor ****/
bool epos_functions::HaltPositionMovement(int motor)
{
    void* keyHandle_local = keyHandle;
    unsigned int error_code = 0;
    bool result = true;
    if((result = VCS_HaltPositionMovement(keyHandle_local, motor, &error_code)) == 0)
    {
        ROS_INFO("ERROR: No se ha podido llegar a la consigna en el motor %d", motor);
        result = false;
        return result;
    }
    return result;
}


/*void epos_functions::move_motor_to_position(void *keyHandle, unsigned short motor, long target_position, unsigned int speed, unsigned int acceleration, unsigned int decelaration, unsigned int *error_code, bool movement_absolute, bool movement_wait_to_start)
{
    int result;
    result = VCS_SetPositionProfile(keyHandle, motor, speed, acceleration, decelaration, error_code);
    result = VCS_MoveToPosition(keyHandle, motor, target_position, movement_absolute, movement_wait_to_start, error_code);
}*/





