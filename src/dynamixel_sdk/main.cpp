#include "controller_dynamixel_sdk.h"

int main(int argc, char **argv)
{

ros::init(argc, argv, "DynamixelSDK_read_wrte");
controller_dynamixel_sdk joint1;
joint1._init_("1");
ros::Rate rate(1);

while(ros::ok()){
joint1.pub_read();

ros::spinOnce();
rate.sleep();
}

}
