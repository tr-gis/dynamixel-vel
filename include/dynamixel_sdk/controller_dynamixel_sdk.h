/*******************************************************************************
* This header files defines the class that can used to interact with single dynamixel motor.

* This builds on top of dynamixel sdk package and hence make sure they are available before 
compiling. Make appropriate changes to CMakeLists.txt if necessary.

* This class was built based upon example/protocol1.0/read_write.cpp.
*******************************************************************************/

#ifndef CONTROLLER_DYNAMIXEL_SDK
#define CONTROLLER_DYNAMIXEL_SDK


#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdlib.h>
#include <stdio.h>


#include "dynamixel_sdk.h"                                  // Uses Dynamixel SDK library
#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <dynamixel_sdk/joint_control.h>

//home/lnt/catkin_ws/devel/include/dynamixel_sdk/joint_control.h

// Control table address
#define ADDR_DXL_CW_ANGLE_LIMIT		6
#define ADDR_DXL_CCW_ANGLE_LIMIT	8
#define ADDR_MX_TORQUE_ENABLE           24                  // Control table address is different in Dynamixel model
#define ADDR_MX_GOAL_POSITION           30
#define ADDR_DXL_GOAL_SPEED		32
#define ADDR_MX_PRESENT_POSITION        36
#define ADDR_DXL_PRESENT_SPEED		38


// Protocol version
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL_ID                          11                   // Dynamixel ID: 1
#define BAUDRATE                        1000000
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_CW_VALUE      		1800                 // Limit for clockwise rotation
#define DXL_CCW_VALUE      		2500                // Limit for counter-clockwise rotation
#define DXL_MOVING_STATUS_THRESHOLD     10                  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b
using namespace std;

class controller_dynamixel_sdk{
private:

int dxl_goal_position;
dynamixel::PortHandler *portHandler;
dynamixel::PacketHandler *packetHandler;
int dxl_comm_result;		
uint8_t dxl_error ;                     // Dynamixel error
uint16_t dxl_present_position;          // Present position
std_msgs::Float64 msg;			//For publishing motor's present position
ros::NodeHandle n1;
ros::NodeHandle n2;


ros::Subscriber jointcommand_sub;

ros::Publisher jointstate_pub;

public:

controller_dynamixel_sdk(); 		//constructor
~controller_dynamixel_sdk(); 		//destructor

void _init_(std::string jointnumber);
void Callback(const dynamixel_sdk::joint_control::ConstPtr& msg_in);


int getch();
int kbhit(void);

int pub_read();
int sub_write_position(float goal);
int sub_write_velocity(float goal);

bool openPort();
bool setBaudRate();
bool enableTorque();
bool setJointMode();
void disableTorque();
void closePort();


};
//void chatterCallback(const std_msgs::Float64::ConstPtr& msg_in);
#endif 
