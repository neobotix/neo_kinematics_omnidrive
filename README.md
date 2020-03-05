# neo_kinematics_omnidrive

## WORK IN PROGRESS! PLEASE DO NOT USE THIS PACKAGE!

# Bringing up the socket can ! 

sudo ip link set can0 type can bitrate 1000000

sudo ip link set can0 up

# Neo Kinematics node needs to be launched! 

roslaunch neo_kinematics_omnidrive neo_kinematics_omnidrive.launch 

# Homing Service

rosservice call /star_homing {}

