#include <SerialClass.hpp>
#include "protocol.h"

unsigned int Protocol::Dynamixel_Receive(unsigned char *Data,unsigned char Data_Length, unsigned char *Return_Data)//ID Address Value
{
    unsigned char  i;
    unsigned char Data2[2];	
    unsigned int Return_State = Dynamixel_State_Error;
    for(i=0;i<Data_Length;i++)
    {
        if(Blk_Num<3)
        {
            if((unsigned char)Data[i] == Dynamixel_Header[Blk_Num])
            {
                Data_Blk[Blk_Num] = (unsigned char)Data[i];
                Blk_Num++;
            }
            else Blk_Num = 0;
        }

        else if(Blk_Num == 3)
        {
            if((unsigned char)Data[i] == Dynamixel_Reserved )
            {
                Data_Blk[Blk_Num] = (unsigned char)Data[i];
                Blk_Num++;
            }
            else Blk_Num = 0;
        }

        else if(Blk_Num == 4)
        {
            if(((unsigned char)Data[i] ==Dynamixel_ID)||(Dynamixel_ID == 256))
            {
                Data_Blk[Blk_Num] = (unsigned char)Data[i];
                Blk_Num++;
            }
            else Blk_Num = 0;
        }

        else if(Blk_Num < 7) //Packet Length
        {

            Data_Blk[Blk_Num] = (unsigned char)Data[i];
            Blk_Num++;
        }

        else if(Blk_Num == 7)
        {
            if((unsigned char)Data[i] ==Dynamixel_Instruction_Return )
            {
                Data_Blk[Blk_Num] = (unsigned char)Data[i];
                Blk_Num++;
            }
            else Blk_Num = 0;
        }

        else if(Blk_Num < 10)
        {
//            if((unsigned char)Data_Buffer[i] == Dynamixel_Error )
//            {
                Data_Blk[Blk_Num] = (unsigned char)Data[i];
                Blk_Num++;
//            }
//            else Blk_Num = 0;
        }

	else if(Blk_Num < (Dynamixel_Value_Forward(Data_Blk,5,6) + 7))
        {
            Data_Blk[Blk_Num] = (unsigned char)Data[i];
            Blk_Num++;
            if(Blk_Num == (Dynamixel_Value_Forward(Data_Blk,5,6) + 7))
            {
                Dynamixel_Value_Dackward(Dynamixel_update_crc(0,Data_Blk,(Dynamixel_Value_Forward(Data_Blk,5,6) + 5)),2,Data2);

                if((Data2[0] == Data_Blk[Data_Blk[5]+5])&&(Data2[1] == Data_Blk[Data_Blk[5]+6]))
                {
                    Return_State = Dynamixel_State_Success;
                    Return_Data[0] = (unsigned char)Data_Blk[4];//ID
                    Return_Data[1] = (unsigned char)Data_Blk[8];
                    Return_Data[2] = Dynamixel_Value_Forward(Data_Blk,5,6)-5;
                    for(char k=0;k<Return_Data[2];k++)
                    {
                        Return_Data[3+k]=(unsigned char)Data_Blk[10+k];
                    }
                }
                Blk_Num =0;
            }
        }
        else if(Blk_Num >= Uart_Blk_Length)
            Blk_Num =0;
    }
	ROS_INFO("the num_blk is %d",Blk_Num);	
     return Return_State;
}

unsigned  char Protocol::Dynamixel_Send(unsigned char Instruction, unsigned short Address,
                                        unsigned int Value, unsigned char Value_Length, unsigned char buf[]) {
    if(SerialClass::id == 0)
        return Dynamixel_State_Error;

    unsigned char i=0,data_lenth=0;
    unsigned char data2[2],data4[4];
    for (i=0;i<3;i++){
        buf[data_lenth++] = Dynamixel_Header[i];
    }
    buf[data_lenth++] = Dynamixel_Reserved;
    buf[data_lenth++] = SerialClass::id;
    //Packet Length
    Dynamixel_Value_Dackward(Value_Length+5, 2, data2);
    for(i=0;i<2;i++)
    {
        buf[data_lenth++] = data2[i];
    }

    buf[data_lenth++] = Instruction;

    Dynamixel_Value_Dackward(Address, 2, data2);
    for(i=0;i<2;i++)
    {
        buf[data_lenth++] = data2[i];
    }

    //Data
    Dynamixel_Value_Dackward(Value, Value_Length, data4);
    for(i=0;i<Value_Length;i++)
    {
        buf[data_lenth++] = data4[i];
    }

    //CRC
    Dynamixel_Value_Dackward(Dynamixel_update_crc(0, buf, data_lenth), 2, data2);
    for(i=0;i<2;i++)
    {
        buf[data_lenth++] = data2[i];
    }
    ROS_INFO_STREAM("the send data is ");
    for(i=0;i<data_lenth;i++){
        ROS_INFO("OX%02x",buf[i]);
    }
    ROS_INFO_STREAM("\n");
    return Dynamixel_State_Success;
}

unsigned short Protocol::Dynamixel_update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr,
                                              unsigned short data_blk_size)
{
    unsigned short i, j;
    for(j = 0; j < data_blk_size; j++)
    {
        i = ((unsigned short)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
        crc_accum = (crc_accum << 8) ^ Dynamixel_crc_table[i];
    }

    return crc_accum;
}

void Protocol::Dynamixel_Value_Dackward(unsigned int Value, unsigned char data_blk_size, unsigned char  Return_data[])
{
    unsigned char i;
    for(i=0;i<data_blk_size;i++)
    {
        Return_data[i] = Value & 0xff;
        Value = Value>>8;
    }
}

unsigned int Protocol::Dynamixel_Value_Forward(unsigned char *data_blk_ptr, unsigned char data_blk_size_start,
                                       unsigned char data_blk_size_end)
{
	unsigned int Value = 0x00000000;
    unsigned char i;
    //if(data_blk_size_start == data_blk_size_end)
        //return Dynamixel_Return_Success;
	for(i=data_blk_size_end;i>=data_blk_size_start;i--)
	{
		Value = Value<<8;
		Value = Value | data_blk_ptr[i]; 

	}
    if(Value> Dynamixel_Return_Position_Value_Max )
    {
       // qDebug("- %x",Value);
        Value -= (0xffff+1);

    }


	return Value;		

}
