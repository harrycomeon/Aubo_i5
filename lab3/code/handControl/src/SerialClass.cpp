#include "SerialClass.hpp"
extern SerialClass * serClass;
unsigned char SerialClass::id = 1;
SerialClass::SerialClass(string ttyName,int Baudrate,int timeout)
{
    this->ttyName = ttyName;
    this->Baudrate = Baudrate;
    this->timeout = timeout;
    this->data = new unsigned char[20];
    this->current_state = State_ID;
    pack_type = READ_ID;
    Address =-1;
    try{
        ser.setPort(ttyName);
        ser.setBaudrate(Baudrate);
        ser.setParity(parity_none);
        ser.setBytesize(eightbits);
        ser.setStopbits(stopbits_one);
        ser.setFlowcontrol(flowcontrol_none);
        Timeout o = Timeout::simpleTimeout(timeout);
        ser.setTimeout(o);
        ser.open();
    }catch (IOException &e){
        ROS_ERROR_STREAM("unable to open port");
        return ;
    }
    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port was opened");
    }

}

void SerialClass::callback(const handControl::hand_control_cmd::ConstPtr msg)
{
    ROS_INFO_STREAM("receive topic");
    serClass->sendCmd(msg);
}

void SerialClass::run()
{
    ros::Rate rate(200);
    rate.sleep();
    sendCmd();
    ros::Rate loop_rate(50);
    data = new unsigned char[10]{0};
    while(ros::ok()){

        ros::spinOnce();
        size_t len = ser.available();
        if(len){
            recvData(len);
        }

        loop_rate.sleep();
    }
}

unsigned char SerialClass::sendCmd(const handControl::hand_control_cmd::ConstPtr msg)
{
    unsigned  char * sendData;
    int len = -1;
    int number = -1;
    if(msg!=NULL){
        ROS_INFO("the cmd is %s,the arg is %d",msg->msgtype,msg->value);
        number = msg->value;
        if(msg->msgtype == open){
            current_state = State_Open;
            pack_type = Enable;
            number = 0;
            Address = GridAddress_Control;
        }else if(msg->msgtype == close){
            current_state = State_Close;
            pack_type = Enable;
            number = 1;
            Address = GridAddress_Control;
        }else if(msg->msgtype == enable){
            pack_type = Enable;
            number = 1;
            current_state = State_Wait;
            Address = GridAddress_Enable;
        }else if(msg->msgtype == disable){
            pack_type = Enable;
            number = 0;
            current_state = State_Wait;
            Address = GridAddress_Enable;
        }
//--
	else if(msg->msgtype == LED){
            pack_type = Enable;
            //number = 0;
            current_state = State_LED;
            Address = GridLEDMode;
        }

	else if(msg->msgtype == getID){
            current_state = State_ID;
            pack_type = Read;
            Address = GridAddress_ID;
        } else if(msg->msgtype == setID){
            pack_type = Write;
            current_state = State_ID_Write;
            Address = GridAddress_ID;
        }else if(msg->msgtype == getHomingOffset){
            pack_type = Read;
            current_state = State_Homing_Offset;
            Address = GridHomingOffset;
        }else if(msg->msgtype == setHomingOffset){
            pack_type = Write;
            current_state = State_Homing_Offset_Write;
            Address = GridHomingOffset;
        }else if(msg->msgtype == getPresentCurrent){
            current_state = State_Present_Current;
            pack_type = Read;
            Address = GridPresentCurrent;
        }else if(msg->msgtype == getPresentPosition){
            current_state = State_Present_Position;
            pack_type = Read;
            Address = GridPresentPosition;
        }
        else{
            return Dynamixel_State_Error;
        }
    }

    switch(pack_type){
        case Read:
		len = Dynamixel_Pack_Length_Read+7;   
		 sendData = new unsigned  char[len]{0};
		if(Address==GridAddress_ID)
		{
			protocol.Dynamixel_Send(Dynamixel_Instruction_Read,Address,4,2,sendData);
		}else
		{
			protocol.Dynamixel_Send(Dynamixel_Instruction_Read,0,62,2,sendData);
		}
        //    len = Dynamixel_Pack_Length_Read+7;   
        //    protocol.Dynamixel_Send(Dynamixel_Instruction_Read,Address,4,2,sendData);
	    
            break;
        case Enable:
            len = Dynamixel_Pack_Length_Enable+7;
            sendData = new unsigned  char[len]{0};
            protocol.Dynamixel_Send(Dynamixel_Instruction_Write,Address,number,1,sendData);
            break;
        case Write:
            len = Dynamixel_Pack_Length_Write+7;
            sendData = new unsigned  char[len]{0};
            protocol.Dynamixel_Send(Dynamixel_Instruction_Write,Address,number,2,sendData);
            break;
        case READ_ID:
            len = 10;
            sendData = new unsigned  char[len]{0};
            for(int i =0;i<10;i++){ sendData[i] = protocol.Dynamixel_Read_ID[i];}
        default:
            break;
    }

    size_t  sendlen = ser.write(sendData, len);
    ROS_INFO("send datalen is %d",sendlen);
    delete  sendData;
    return Dynamixel_State_Success;
}

unsigned char SerialClass::recvData(size_t len) {
    unsigned char * data = new unsigned char[len]{0};
    size_t recvlen = ser.read(data,len);
    if(current_state == State_ID){
        SerialClass::id =data[4];
    }
    ROS_INFO("rece data,the data len is %d;recvlen is %d,id is %d",len,recvlen,SerialClass::id);
    for(int i=0;i<len;i++){
        ROS_INFO("OX%02x",data[i]);
    }
	
    ROS_INFO_STREAM("\n");
	
    if(protocol.Dynamixel_Receive(data,len,REC)==Dynamixel_State_Success){

	int Value_return = 0;
	
        switch (current_state) {
            case State_Present_Position:
		Value_return = protocol.Dynamixel_Value_Forward(REC, GridPresentPosition+3, GridPresentPosition+2+2);
		ROS_INFO("the present position is %d",Value_return);
                break;

            case State_Present_Current:
		Value_return = protocol.Dynamixel_Value_Forward(REC, GridPresentCurrent+3, GridPresentCurrent+2+2);
		ROS_INFO("the present current is %d",Value_return);
                break;

            case State_ID:
                ROS_INFO("the present id is %d",REC[0]);//SerialClass::id
                break;

            case State_ID_Write:
                ROS_INFO("the setid is %d",SerialClass::id);
                break;

            case State_Homing_Offset:
		Value_return = protocol.Dynamixel_Value_Forward(REC, GridHomingOffset+3, GridHomingOffset+2+2);
                ROS_INFO("the present homingOffset is %d",Value_return);
                break;
            case State_Homing_Offset_Write:
		Value_return = protocol.Dynamixel_Value_Forward(REC, GridHomingOffset+3, GridHomingOffset+2+2);
                ROS_INFO("the present homingOffset is %d",Value_return);
                break;

            case State_Torque_Enable:
                ROS_INFO("enable the hand");
                break;
            case State_Close:
                ROS_INFO("the hand is loose");
                break;
            case State_Open:
                ROS_INFO("the hand is tight");

            case State_Wait:
                ROS_INFO("setting the param");
                break;
	    case State_LED:
		ROS_INFO("setting the LED");
                break;
            default:
                break;
        }

    }else{
        ROS_INFO("recv data error");
    }

}
