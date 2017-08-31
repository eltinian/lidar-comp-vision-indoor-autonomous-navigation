# TEENSY Firmware

> The teensyboard 3.2 is used as the electronic speed control (ESC)

## Setting up the firmware

### 1 Install the teensyduino software
Follow the insturctions here: https://www.pjrc.com/teensy/td_download.html

### 2 Setup `rosserial`

```bash
    sudo apt install ros-kinetic-rosserial-arduino
    sudo apt install ros-kinetic-rosserial
```

### 3 Setup libraries

> This step involves in generating the required ROS libraries for the ROS project. This includes generating the message types defined in the ROS package.

Open a terminal:

```
# source the ROS workspace
source devel/setup.bash

# Generate the arduino libraries. 
cd <sketchbook>/libraries  # See note below
rm -rf ros_lib
rosrun rosserial_arduino make_libraries .
```

**NOTE**: `<sketchbook>` is the location of the default Arduino libraries folder. Generally located under `~/Arduino`

### 4  Program the Teensy 

Choose Teensy 3.2/3.1 in Tools menu of arduino software. Also make sure settings are configured to : "serial" and "96 Mhz optimized".

After choosing the appropriate port, your arduino should be ready to dump the firmware.