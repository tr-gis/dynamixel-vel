/*******************************************************************************
* Copyright (c) 2016, ROBOTIS CO., LTD.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* * Neither the name of ROBOTIS nor the names of its
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/* Author: Ryu Woon Jung (Leon) */

//
// *********     Bulk Read and Bulk Write Example      *********
//
//
// Available Dynamixel model on this example : All models using Protocol 2.0
// This example is tested with two Dynamixel PRO 54-200, and an USB2DYNAMIXEL
// Be sure that Dynamixel PRO properties are already set as %% ID : 1 and 2 / Baudnum : 1 (Baudrate : 57600)
//

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
//!!!!!!xxxxxxxxxxx!!!!!!!!!xxxxxxxxxxxxx!!!!add the custom message header file!!!!!!!xxxxxxxxxxxxxxxxxxxxx!!!!!!!!!!!!!!!!!!!!!!xxxxxxxxxx*
#include "std_msgs/String.h"
#include <ros/callback_queue.h>

// Control table address
#define ADDR_PRO_TORQUE_ENABLE          562                 // Control table address is different in Dynamixel model
#define ADDR_PRO_LED_RED                563
#define ADDR_PRO_GOAL_POSITION          596
#define ADDR_PRO_PRESENT_POSITION       611

// Data Byte Length
#define LEN_PRO_LED_RED                 1
#define LEN_PRO_GOAL_POSITION           4
#define LEN_PRO_PRESENT_POSITION        4

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL1_ID                         1                   // Dynamixel#1 ID: 1
#define DXL2_ID                         2                   // Dynamixel#2 ID: 2
#define BAUDRATE                        115200
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque

#define DXL1_MINIMUM_POSITION_VALUE      -250950           // Dynamixel will rotate between this value 1:-250950 to +250950
#define DXL1_MAXIMUM_POSITION_VALUE      250950 
#define DXL2_MINIMUM_POSITION_VALUE      0
#define DXL2_MAXIMUM_POSITION_VALUE      250950 												//2: 0 to 250950
												//3: -250950 to 0
												//4: -101265 to +101268	
												//5: -75950 to +75950
												//6:-151900 to +151900
           
#define DXL_MOVING_STATUS_THRESHOLD     20                  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b

//global variables shared between callback function and main()

int dxl_comm_result = COMM_TX_FAIL;             // Communication result
  bool dxl_addparam_result = false;               // addParam result
  bool dxl_getdata_result = false;                // GetParam result
  
  uint8_t dxl_error = 0;                          // Dynamixel error
  uint8_t param_goal_position[4];
  int dxl_goal_position;          		  // Goal position
  int32_t dxl1_present_position = 0;              // Present position 
  int32_t dxl2_present_position = 0;

void dynamixelsdk_callback(/*add_message_type_here ConstPtr& msg*/)//******************modify_this_bit*************************************
{
   switch(msg->joint_number):
	case 1://single joint mode operation of joint 1
		dxl_goal_position=/*msg->position[0]*/
		assignposition();
		// Add parameter storage for Dynamixel#1 goal position
		add_goal(DXL1_ID);//************************************check here if #define DXL1_ID is of int type or not
		dxl_goal_position=read_data(DXL2_ID); // assign current joint value as goal
		assignposition();		
		// Add parameter storage for Dynamixel#2 goal position
		add_goal(DXL2_ID);
		bulk_write();
				
		break;
	case 2:
		//single joint mode operation of joint 2
		dxl_goal_position=read_data(DXL1_ID);// assign current joint value as goal
		assignposition();
		// Add parameter storage for Dynamixel#1 goal position
		add_goal(DXL1_ID);//************************************check here if #define DXL1_ID is of int type or not
		dxl_goal_position=/*msg->position[1]*/ 
		assignposition();		
		// Add parameter storage for Dynamixel#2 goal position
		add_goal(DXL2_ID);
		bulk_write();
		
		break;
	case 7:
		//msg->joint_number 7 is used to identify multi-joint operation mode
		dxl_goal_position=/*msg->position[0]*/
		if((DXL1_MINIMUM_POSITION_VALUE<dxl_goal_position) && (dxl_goal_position < DXL1_MAXIMUM_POSITION_VALUE)){
		assignposition();
		add_goal(DXL1_ID);
		}
		else{
		dxl_goal_position=read_data(DXL1_ID);
		assignposition();
		add_goal(DXL1_ID);
		}
		
		dxl_goal_position=/*msg->position[1]*/ 
		if((DXL2_MINIMUM_POSITION_VALUE<dxl_goal_position) && (dxl_goal_position < DXL2_MAXIMUM_POSITION_VALUE)){
		assignposition();
		add_goal(DXL2_ID);
		}
		else{
		dxl_goal_position=read_data(DXL2_ID);
		assignposition();
		add_goal(DXL2_ID);
		}
		bulk_write();
		
		break;
	default:
	ROS_INFO("Invalid packet code used for input");
		


    

    
    // Add parameter storage for Dynamixel#1 goal position
    dxl_addparam_result = groupBulkWrite.addParam(DXL1_ID, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION, param_goal_position);
    if (dxl_addparam_result != true)
    {
      fprintf(stderr, "[ID:%03d] groupBulkWrite addparam failed", DXL1_ID);
      return 0;
    }

    // Add parameter storage for Dynamixel#2 goal position
    dxl_addparam_result = groupBulkWrite.addParam(DXL2_ID, ADDR_PRO_LED_RED, LEN_PRO_LED_RED, &dxl_led_value[index]);//****modify here
    if (dxl_addparam_result != true)
    {
      fprintf(stderr, "[ID:%03d] groupBulkWrite addparam failed", DXL2_ID);
      return 0;
    }

    // Bulkwrite goal positions
    dxl_comm_result = groupBulkWrite.txPacket();
    if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));

    // Clear bulkwrite parameter storage
    groupBulkWrite.clearParam();

    do
    {
      // Bulkread present position and LED status
      dxl_comm_result = groupBulkRead.txRxPacket();
      if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));

      // Check if groupbulkread data of Dynamixel#1 is available
      dxl_getdata_result = groupBulkRead.isAvailable(DXL1_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
      if (dxl_getdata_result != true)
      {
        fprintf(stderr, "[ID:%03d] groupBulkRead getdata failed", DXL1_ID);
        return 0;
      }

      // Check if groupbulkread data of Dynamixel#2 is available
      dxl_getdata_result = groupBulkRead.isAvailable(DXL2_ID, ADDR_PRO_LED_RED, LEN_PRO_LED_RED);
      if (dxl_getdata_result != true)
      {
        fprintf(stderr, "[ID:%03d] groupBulkRead getdata failed", DXL2_ID);
        return 0;
      }

      // Get present position motor 1
      dxl1_present_position = groupBulkRead.getData(DXL1_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);

      // Get present position of motor 2
      dxl2_present_position = groupBulkRead.getData(DXL2_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);

      printf("[ID:%03d] Present Position : %d \t [ID:%03d] LED Value: %d\n", DXL1_ID, dxl1_present_position, DXL2_ID, dxl2_led_value_read);

    }while((abs(dxl_goal_position - dxl1_present_position) || (abs(dxl_goal_position - dxl2present_position))> DXL_MOVING_STATUS_THRESHOLD);

    dxl_goal_position+=1000;

}

void assignposition() {

    param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(dxl_goal_position));
    param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(dxl_goal_position));
    param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(dxl_goal_position));
    param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(dxl_goal_position));
  }

void add_goal(int DXL_ID){
// Add parameter storage for Dynamixel#1 goal position
    dxl_addparam_result = groupBulkWrite.addParam(DXL_ID, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION, param_goal_position);
    if (dxl_addparam_result != true)
    {
      fprintf(stderr, "[ID:%03d] groupBulkWrite addparam failed", DXL_ID);
      
    }
}

int read_data(int DXL_ID){
 dxl_getdata_result = groupBulkRead.isAvailable(DXL_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
      if (dxl_getdata_result != true)
      {
        fprintf(stderr, "[ID:%03d] groupBulkRead getdata failed", DXL_ID);
        return 0;
      }
      else
	return groupBulkRead.getData(DXL_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
}

void bulk_write(){
dxl_comm_result = groupBulkWrite.txPacket();
    if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));

    // Clear bulkwrite parameter storage
    groupBulkWrite.clearParam();
}

int getch()
{
#if defined(__linux__) || defined(__APPLE__)
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
#elif defined(_WIN32) || defined(_WIN64)
  return _getch();
#endif
}

int kbhit(void)
{
#if defined(__linux__) || defined(__APPLE__)
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
#elif defined(_WIN32) || defined(_WIN64)
  return _kbhit();
#endif
}

int main(int argc, char **argv)
{
  ros::init(argc, **argv, "DynamixelSDK_bulk_read_wrte");
  ros::NodeHandle n;
  ros::Rate r(4); 
  ros::Subscriber sub = n.subscribe("chatter", 1000, dynamixelsdk_callback); //****************modify this bit**********************
  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Initialize GroupBulkWrite instance
  dynamixel::GroupBulkWrite groupBulkWrite(portHandler, packetHandler);

  // Initialize GroupBulkRead instance
  dynamixel::GroupBulkRead groupBulkRead(portHandler, packetHandler);

  
  // Open port
  if (portHandler->openPort())
  {
    printf("Succeeded to open the port!\n");
  }
  else
  {
    printf("Failed to open the port!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }

  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE))
  {
    printf("Succeeded to change the baudrate!\n");
  }
  else
  {
    printf("Failed to change the baudrate!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }



  // Enable Dynamixel#1 Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    printf("%s\n", packetHandler->getRxPacketError(dxl_error));
  }
  else
  {
    printf("Dynamixel#%d has been successfully connected \n", DXL1_ID);
  }

  // Enable Dynamixel#2 Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    printf("%s\n", packetHandler->getRxPacketError(dxl_error));
  }
  else
  {
    printf("Dynamixel#%d has been successfully connected \n", DXL2_ID);
  }

  // Add parameter storage for Dynamixel#1 present position
  dxl_addparam_result = groupBulkRead.addParam(DXL1_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
  if (dxl_addparam_result != true)
  {
    fprintf(stderr, "[ID:%03d] grouBulkRead addparam failed", DXL1_ID);
    return 0;
  }

  // Add parameter storage for Dynamixel#2 present position
  dxl_addparam_result = groupBulkRead.addParam(DXL2_ID, DXL1_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
  if (dxl_addparam_result != true)
  {
    fprintf(stderr, "[ID:%03d] grouBulkRead addparam failed", DXL2_ID);
    return 0;
  }

  while(ros::ok())
  {
   ros::spin();
   r.sleep();
    
  }

  // Disable Dynamixel#1 Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    printf("%s\n", packetHandler->getRxPacketError(dxl_error));
  }

  // Disable Dynamixel#2 Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    printf("%s\n", packetHandler->getRxPacketError(dxl_error));
  }

  // Close port
  portHandler->closePort();

  return 0;
}
