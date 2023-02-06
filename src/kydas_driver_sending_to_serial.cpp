#include "kydas_driver/kydas_driver.h"

void KydasDriverNode::setSpeed(int value, unsigned char controlMode){
    if(!m_isConnected){
        ROS_WARN("can't set motor command: driver not connected");
        return;
    }
    else if(!m_isEnabled){
        ROS_WARN("can't set motor command: motor not enabled");
        return;
    }
    char* valueInBytes = static_cast<char*>(static_cast<void*>(&value));
    std::vector<unsigned char> command ={CONTROL_HEADER,0,0,0,0,0,0,0};
    
    //Placing the command mode
    //TODO estudar os diferentes modos e os efeitos no motor
    command[1] = controlMode;// -- Precisamos verificar se realmente nao tem que modificar nada quando em modo serial
    //Placing the value
    command[4] = valueInBytes[3]; 
    command[5] = valueInBytes[2]; 
    command[6] = valueInBytes[1]; 
    command[7] = valueInBytes[0]; 
    m_messagesToSend.push(command);

    //int result = RS232_SendBuf(m_cport_nr, command,commandSize);
    ROS_DEBUG_NAMED(DEBUGGER_NAME_COMMAND_SENT, "seting motor value [%d] on mode [%d]",value, (int)controlMode);
}

void KydasDriverNode::requestQueryData(unsigned char command){
    if(!m_isConnected){
        ROS_WARN("can't request query data: driver not connected");
        return;
    }
    std::vector<unsigned char> queryCommand = {QUERY_HEADER,0,0,0,0,0,0,0};
    queryCommand[1] = command;
    //int result = RS232_SendBuf(m_cport_nr, queryCommand,commandSize);
    //ROS_DEBUG_NAMED(DEBUGGER_NAME_COMMAND_SENT, "requesting data [%d]", command);
    //
    //std::string s = displayMessage(queryCommand, commandSize);
    //const char* cstr = s.c_str();
    //ROS_DEBUG_NAMED(DEBUGGER_NAME_MESSAGE_SENT, "message = [%s]", cstr);
    m_messagesToSend.push(queryCommand);
}

int KydasDriverNode::sendNextMessage(){
    if(m_messagesToSend.size() <= 0){
        return 0;
    }

    std::vector<unsigned char> message = m_messagesToSend.front();
    int result = RS232_SendBuf(m_cport_nr, &message[0], message.size());   
    std::string s = displayMessage(&message[0], message.size());
    const char* cstr = s.c_str();
    ROS_DEBUG_NAMED(DEBUGGER_NAME_MESSAGE_SENT, "message = [%s]", cstr);

    m_messagesToSend.pop(); 
    return result;
}