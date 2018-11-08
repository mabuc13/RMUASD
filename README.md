# RMUASD

In order to launch the gazebo simulation, you have to source the px4_init file. You must also run the serial2udp.sh script in order to set up the bridge between the serial output from mavlink lora and gazebo. The launch file for mavlink lora also has to be set to point at the serial port /tmp/ttyMAVLINK

roslaunch px4 posix_sitl.launch
or
roslaunch px4 posix_sitl.launch gui:=false
