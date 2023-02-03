#include "kydas_driver/kydas_driver.h"

//TODO fazer um ROS Timer para que isso seja enviado a cada 500 ms
int KydasDriverNode::setMotorCommand(int value, unsigned char controlMode){
    if(!m_isConnected){
        ROS_WARN("can't set motor command: driver not connected");
        return -1;
    }
    else if(!m_isEnabled){
        ROS_WARN("can't set motor command: motor not enabled");
        return -1;
    }
    char* valueInBytes = static_cast<char*>(static_cast<void*>(&value));
    unsigned char command[]={CONTROL_HEADER,0,0,0,0,0,0,0};
    const int commandSize = 8;
    
    //Placing the command mode
    //TODO estudar os diferentes modos e os efeitos no motor
    command[1] = controlMode;// -- Precisamos verificar se realmente nao tem que modificar nada quando em modo serial
    //Placing the value
    command[4] = valueInBytes[3]; 
    command[5] = valueInBytes[2]; 
    command[6] = valueInBytes[1]; 
    command[7] = valueInBytes[0]; 
    int result = RS232_SendBuf(m_cport_nr, command,commandSize);
    ROS_WARN("seting motor value [%d] on mode [%d]",value, (int)controlMode);
    displayMessage(command, commandSize);
    return result;
}

int KydasDriverNode::enableMotor(){
    if(!m_isConnected){
        ROS_WARN("can't enable motor: driver not connected");
        return -1;
    }
    unsigned char enableCommand[]={CONTROL_HEADER,1,0,0,0,0,0,0};
    const int commandSize = 8;
    int result = RS232_SendBuf(m_cport_nr, enableCommand,commandSize);
    ROS_DEBUG("enabling motor");
    displayMessage(enableCommand, commandSize);
    return result;
}

int KydasDriverNode::disableMotor(){
    if(!m_isConnected){
        ROS_WARN("can't disable motor: driver not connected");
        return -1;
    }
    unsigned char enableCommand[]={CONTROL_HEADER,0,0,0,0,0,0,0};
    const int commandSize = 8;
    int result = RS232_SendBuf(m_cport_nr, enableCommand,commandSize);
    ROS_DEBUG("disabling motor");
    displayMessage(enableCommand, commandSize);
    return result;
}

int KydasDriverNode::requestQueryData(unsigned char command){
    if(!m_isConnected){
        ROS_WARN("can't request query data: driver not connected");
        return -1;
    }
    unsigned char queryCommand[]={QUERY_HEADER,0,0,0,0,0,0,0};
    const int commandSize = 8;
    queryCommand[1] = command;
    int result = RS232_SendBuf(m_cport_nr, queryCommand,commandSize);
    ROS_DEBUG("requesting data [%d]", command);
    displayMessage(queryCommand, commandSize);
    return result;
}