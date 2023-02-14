#include "kydas_driver/kydas_driver.h"

void KydasDriver::setSpeed(int value, unsigned char controlMode){
    if(!isConnected){
        ROS_WARN("can't set motor command: driver not connected");
        return;
    }
    char* valueInBytes = static_cast<char*>(static_cast<void*>(&value));
    std::array<unsigned char, 8> command ={CONTROL_HEADER,0,0,0,0,0,0,0};
    
    //Placing the command mode
    if(value == 0){
        command[1] = 0;
    }
    else{
        command[1] = controlMode;
    }
    //Placing the value
    command[4] = valueInBytes[3]; 
    command[5] = valueInBytes[2]; 
    command[6] = valueInBytes[1]; 
    command[7] = valueInBytes[0];     
    sendMessage(command);
    ROS_DEBUG_NAMED(DEBUGGER_NAME_COMMAND_SENT, "seting motor value [%d] on mode [%d]",value, (int)controlMode);
}

void KydasDriver::requestQueryData(unsigned char command){
    if(!isConnected){
        ROS_WARN("can't request query data: driver not connected");
        return;
    }
    std::array<unsigned char, 8> queryCommand = {QUERY_HEADER,0,0,0,0,0,0,0};
    queryCommand[1] = command;
    sendMessage(queryCommand);
    ROS_DEBUG_NAMED(DEBUGGER_NAME_DATA_REQUEST_SENT, "requesting data of type [%d]", (int) command);
}

int KydasDriver::sendMessage(std::array<unsigned char> msg){
    if(msg.size() <= 0){
        return 0;
    }
    int result = RS232_SendBuf(m_cport_nr, &msg[0], msg.size());   
    std::string s = displayMessage(&msg[0], msg.size());
    const char* cstr = s.c_str();
    ROS_DEBUG_NAMED(DEBUGGER_NAME_MESSAGE_SENT, "message = [%s]", cstr);
    return result;
}