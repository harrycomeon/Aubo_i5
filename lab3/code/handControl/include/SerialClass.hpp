#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include "handControl/hand_control_cmd.h"
#include <string>
#include "protocol.h"

using namespace serial;
using namespace std;

class SerialClass {
public:
    enum CMD_STATE{
        State_Present_Current =1,
        State_Present_Position,
        State_ID,
        State_Max_Current,
        State_Homing_Offset_Write,
        State_Homing_Offset,
        State_Wait,
        State_ID_Write,
        State_Torque_Enable,
        State_Open,
        State_Close,
	State_LED
    };
    enum PACK_TYPE{
        Read =0,
        Write,
        Enable,
        READ_ID
    };
    const string open = "Open";
    const string close = "Close";
    const string enable = "Enable";
    const string disable = "Disable";
    const string getHomingOffset = "GHO";
    const string setHomingOffset = "SHO";
    const string LED = "LED";
    const string getPresentPosition = "GPP";
    const string getPresentCurrent = "GPC";
    const string setID = "SID";
    const string getID = "GID";

private:
    Serial ser;
    string ttyName;
    int Baudrate;
    int timeout;
    int value;
unsigned char REC[512];
    Protocol protocol;
    unsigned char *data;
    CMD_STATE current_state;
    PACK_TYPE pack_type;
    unsigned char  Address;

public:
    static unsigned char id;

    static void callback(const handControl::hand_control_cmd::ConstPtr msg);

    SerialClass(string ttyName, int Baudrate, int timeout = 1000);

    unsigned char sendCmd(const handControl::hand_control_cmd::ConstPtr msg=NULL);

    unsigned char recvData(size_t len);

    void run();


};
