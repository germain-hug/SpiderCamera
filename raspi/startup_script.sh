#! /bin/bash

PATH=/sbin:/usr/sbin:/bin:/usr/bin:/usr/local/bin:/home/pi/catkin_ws/src
source /opt/ros/kinetic/setup.bash
source /home/pi/catkin_ws/devel/setup.bash
export PATH=$PATH:/opt/arduino-1.8.1/

cd /home/pi/catkin_ws/src/
export ROS_MASTER_URI=http://172.20.10.2:11311 # Master IP
export ROS_IP=172.20.10.5 #Slave IP

#nb : change ~/.profile as well ! 

#echo  "---------------------------------------"
#echo  "---- Git Pull from SpiderCam_Raspi ----"
#echo  "---------------------------------------"
#echo " "
#cd /home/pi/catkin_ws/src && git pull origin master
#echo " "


#echo "----------------------------------"
#echo "---- Uploading Arduino Sketch ----"
#echo "----------------------------------"
#echo " "
#cd /home/pi/catkin_ws/src/arduino_sketch && arduino --upload ax-12w_driver/ax-12w_driver.ino
#echo " "


echo "------------------------------"
echo "---- Starting Safety Node ----"
echo "------------------------------"
echo " "
rosrun cmd_vel_controller safety_check.py &
sleep 5
echo " Done "
echo " "


echo "----------------------------"
echo "---- Starting Rosserial ----"
echo "----------------------------"
echo " "
rosrun rosserial_python serial_node.py /dev/ttyUSB0 _baud:=1000000
sleep 15
