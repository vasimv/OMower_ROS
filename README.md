This is support package for OMower_Simple compiled with ROS. The package includes:

omower_gateway - acts as TCP server for pfodApp/Modbus connections to issue commands, also translates RTKLIB's messages to OMower
omower_seeker.py - gets image stream from camera and detects chessboard on it to control movement when parking
omower_gps_localization.py - translates internal OMower's internal GPS/IMU format to ROS's rviz program to vizualize movements


HOW TO COMPILE

First, don't forget to compile OMower_Simple with ROS support (USE_ROS in omower-defs.h).

Obtain and compile sources of the package:

source /opt/ros/<distro>/setup.bash
source ~/catkin_ws/devel/setup.bash
cd ~/catkin_ws/src
git clone https://github.com/vasimv/OMower_ROS omower_gateway
cd ..
catkin_make


HOW TO RUN

Start rosserial on port connected to the OMower (for OMower v3 board it is /dev/ttyS2 on Orange PI Zero):

rosrun rosserial_python serial_node.py _port:=/dev/ttyS2 _baud:=460800


Start omower_gateway (RTKLIB's output socket should be at port 8990):

rosrun omower_gateway omower_gateway


(Optional) Start omower_seeker.py and omower_gps_localization.py:

rosrun omower_gateway omower_seeker.py

rosrun omower_gateway omower_gps_localization.py


Connect with pfodApp to the OMower on port 8991 and you should get main menu.


[[https://github.com/vasimv/OMower_ROS/blob/master/OMower-ROS.jpg]]

