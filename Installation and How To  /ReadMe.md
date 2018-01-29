Motor ID , Buad rate are set in the file [controller_dynamixel_sdk.h](https://github.com/tr-gis/dynamixel-vel/blob/master/include/dynamixel_sdk/controller_dynamixel_sdk.h) located at:
/home/user_name/workspace_name/src/dynamixel_sdk/include/dynamixel_sdk

The Joint ID is set in the file  [main.cpp ](https://github.com/tr-gis/dynamixel-vel/blob/master/src/dynamixel_sdk/main.cpp)located at:
/home/user_name/workspace_name/src/dynamixel_sdk/src/dynamixel_sdk

For details on Baud rate values refer to spec sheet of the motor.

The angle limits can be set with DXL_CW_VALUE and DXL_CCW_VALUE flags located in  [controller_dynamixel_sdk.h](https://github.com/tr-gis/dynamixel-vel/blob/master/include/dynamixel_sdk/controller_dynamixel_sdk.h).



