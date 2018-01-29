/*******************************************************************************
* This source file contains definition of the class defined in include/controller_dynamixel_sdk.h


*******************************************************************************/
#include "controller_dynamixel_sdk.h"

controller_dynamixel_sdk::controller_dynamixel_sdk(){

portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
int dxl_comm_result=0;		
uint8_t dxl_error=0;                     
uint16_t dxl_present_position=0;
}

void controller_dynamixel_sdk::_init_(std::string jointnumber){
jointstate_pub = n1.advertise<std_msgs::Float64>("Joint"+jointnumber+"_State", 1000);
jointcommand_sub = n2.subscribe("Joint"+jointnumber+"_Goal", 1000, &controller_dynamixel_sdk::Callback,this);

if(openPort()){
if(setBaudRate()){
if(enableTorque()){
if(setJointMode()){
	std::string jointmsg="Joint"+jointnumber+" initialized successfully";
ROS_INFO(jointmsg.c_str());
			}
 
		}
	}
}
}


void controller_dynamixel_sdk::Callback(const dynamixel_sdk::joint_control::ConstPtr& msg_in){
//ROS_INFO("callback triggered");
sub_write_velocity(msg_in->goal_velocity);
sub_write_position(msg_in->goal_position);

}

int controller_dynamixel_sdk::getch(){
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

int controller_dynamixel_sdk::kbhit(void){
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

int controller_dynamixel_sdk::pub_read(){
// Read present position and publish it
      dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID, ADDR_MX_PRESENT_POSITION, &dxl_present_position, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS)
      {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	return 1;
      }
      else if (dxl_error != 0)
      {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
	return 1;
      }

      //printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", DXL_ID, dxl_goal_position, dxl_present_position);
      msg.data=dxl_present_position;
    jointstate_pub.publish(msg);
	return 0;
}

int controller_dynamixel_sdk::sub_write_position(float goal){

dxl_goal_position=goal;
//ROS_INFO("callback triggered");
 // Write goal position
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_GOAL_POSITION, dxl_goal_position, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	return 1;
    }
    else if (dxl_error != 0)
    {
      printf("%s\n", packetHandler->getRxPacketError(dxl_error));
	return 1;
    }
return 0;

}

int controller_dynamixel_sdk::sub_write_velocity(float goal){

dxl_goal_position=goal;
//ROS_INFO("callback triggered");
 // Write goal velocity
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_DXL_GOAL_SPEED, dxl_goal_position, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	return 1;
    }
    else if (dxl_error != 0)
    {
      printf("%s\n", packetHandler->getRxPacketError(dxl_error));
	return 1;
    }
return 0;

}

bool controller_dynamixel_sdk::openPort(){
if (portHandler->openPort())
  {
    printf("Succeeded to open the port!\n");
	return true;

  }
  else
  {
    printf("Failed to open the port!\n");
    printf("Press any key to terminate...\n");
    getch();
	return false;
	

  }
}

bool controller_dynamixel_sdk::setBaudRate(){
if (portHandler->setBaudRate(BAUDRATE))
  {
    printf("Succeeded to change the baudrate!\n");
	return true;

  }
  else
  {
    printf("Failed to change the baudrate!\n");
    printf("Press any key to terminate...\n");
    getch();
	return false;
   
  }
}

bool controller_dynamixel_sdk::enableTorque(){
// Enable Dynamixel Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	return false;
  }
  else if (dxl_error != 0)
  {
    printf("%s\n", packetHandler->getRxPacketError(dxl_error));
	return false;
  }
  else
  {
    printf("Dynamixel has been successfully connected \n");
	return true;
	
  }
}

bool controller_dynamixel_sdk::setJointMode(){

// Enable Joint Mode on dynamixel
bool cwset;
  dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_DXL_CW_ANGLE_LIMIT, DXL_CW_VALUE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	return false;
  }
  else if (dxl_error != 0)
  {
    printf("%s\n", packetHandler->getRxPacketError(dxl_error));
	return false;
  }
  else
  {
    printf("Dynamixel CW angle set to %d successfully \n",DXL_CW_VALUE);
	cwset=true;
  }

dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_DXL_CCW_ANGLE_LIMIT, DXL_CCW_VALUE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	return false;
  }
  else if (dxl_error != 0)
  {
    printf("%s\n", packetHandler->getRxPacketError(dxl_error));
	return false;
  }
  else
  {
    printf("Dynamixel CCW angle set to %d successfully \n",DXL_CCW_VALUE);
	if(cwset)
		return true;
  }
}

void controller_dynamixel_sdk::disableTorque(){
// Disable Dynamixel Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    printf("%s\n", packetHandler->getRxPacketError(dxl_error));
  }
}

void controller_dynamixel_sdk::closePort(){
// Close port
  portHandler->closePort();

}

controller_dynamixel_sdk::~controller_dynamixel_sdk(){
disableTorque();
closePort();
}


