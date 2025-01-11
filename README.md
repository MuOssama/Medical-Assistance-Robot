<<<<<<< HEAD
# Medical-Assistance-Robot
This is complete medical assistance robot runs on ROS which contains smart medical watch node, sever node carry patient data , motion node, dispensing medicine (syrup and tablets) and call for emergency if patient is in danger  
# Medical Robot Setup Instructions

This guide explains how to set up a medical robot system with ROS Kinetic on Ubuntu 16.04, consisting of a Python server node and two Arduino nodes (motion and dispenser).

## Prerequisites

- Ubuntu 16.04
- ROS Kinetic
- Arduino IDE
- Initialized ROS workspace named "medical_robot_ws"

## 1. Create Package Structure

```bash
cd ~/medical_robot_ws/src
catkin_create_pkg medical_robot rospy std_msgs
cd medical_robot
mkdir scripts
mkdir web
mkdir arduino
```

## 2. Set Up Server Node

```bash
# Create and make executable the server node
cd ~/medical_robot_ws/src/medical_robot/scripts
touch server_node.py
chmod +x server_node.py

# Create web directory structure
cd ../web
mkdir static
touch static/index.html
```

Copy the provided code:
- Server code (from paste-2.txt) into `scripts/server_node.py`
- HTML code (from paste.txt) into `web/static/index.html`

## 3. Set Up Arduino Nodes

```bash
cd ~/medical_robot_ws/src/medical_robot/arduino
touch motion_node.ino
touch dispenser_node.ino
```

Copy the provided code:
- Motion code (from paste-4.txt) into `arduino/motion_node.ino`
- Dispenser code (from paste-5.txt) into `arduino/dispenser_node.ino`

## 4. Install Required Packages

```bash
# Install ROS-Arduino bridge
sudo apt-get update
sudo apt-get install ros-kinetic-rosserial-arduino
sudo apt-get install ros-kinetic-rosserial

# Install Arduino IDE
sudo apt-get install arduino arduino-core

# Install Python dependencies
sudo apt-get install python-pip python-flask python-flask-cors sqlite3
```

## 5. Install ROS Libraries for Arduino

```bash
# Create ROS libraries for Arduino
cd ~/medical_robot_ws/src/medical_robot/arduino
rm -rf ros_lib  # Remove if exists
rosrun rosserial_arduino make_libraries.py .
```

## 6. Configure Arduino IDE

1. Open Arduino IDE
2. Go to File -> Preferences
3. Set Sketchbook location to: `~/medical_robot_ws/src/medical_robot/arduino`
4. Restart Arduino IDE
5. Open `motion_node.ino` and `dispenser_node.ino`
6. Select your board type and port
7. Upload both sketches to their respective Arduino boards

## 7. Create Launch File

```bash
cd ~/medical_robot_ws/src/medical_robot
mkdir launch
cd launch
touch medical_robot.launch
```

Add this content to `medical_robot.launch`:
```xml
<launch>
    <!-- Launch the server node -->
    <node name="server_node" pkg="medical_robot" type="server_node.py" output="screen" />
    
    <!-- Launch rosserial for both Arduino boards -->
    <node name="motion_arduino" pkg="rosserial_python" type="serial_node.py">
        <param name="port" value="/dev/ttyACM0"/>
        <param name="baud" value="57600"/>
    </node>
    
    <node name="dispenser_arduino" pkg="rosserial_python" type="serial_node.py">
        <param name="port" value="/dev/ttyACM1"/>
        <param name="baud" value="57600"/>
    </node>
</launch>
```

## 8. Build Workspace

```bash
cd ~/medical_robot_ws
catkin_make
source devel/setup.bash
```

## 9. Set Up Arduino Permissions

```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER

# Give permissions to Arduino ports
sudo chmod a+rw /dev/ttyACM0
sudo chmod a+rw /dev/ttyACM1
```

Note: You might need to log out and back in for group changes to take effect.

## 10. Run the System

```bash
roslaunch medical_robot medical_robot.launch
```

## Troubleshooting

1. Check Arduino port assignments:
```bash
ls /dev/ttyACM*
```
Update the ports in the launch file if they're different from ACM0 and ACM1.

2. If you can't upload to Arduino:
- Check if you have the correct permissions
- Verify the correct board and port are selected in Arduino IDE
- Try unplugging and replugging the Arduino boards

3. If ROS nodes aren't communicating:
- Check that roscore is running
- Verify all nodes are running with `rosnode list`
- Check topic connections with `rostopic list` and `rostopic echo`

4. Database issues:
- Ensure SQLite3 is installed
- Check database permissions in the server directory

## Project Structure

Your final project structure should look like this:
```
medical_robot_ws/
└── src/
    └── medical_robot/
        ├── scripts/
        │   └── server_node.py
        ├── web/
        │   └── static/
        │       └── index.html
        ├── arduino/
        │   ├── motion_node.ino
        │   └── dispenser_node.ino
        └── launch/
            └── medical_robot.launch
```
=======
# Medical Assistance Robot

This project is a comprehensive medical assistance robot developed using the Robot Operating System (ROS). It integrates various functionalities to support patient care, including health monitoring, data management, mobility, medication dispensing, and emergency response.

## Repository Structure

- **Despenser/**: Manages medication dispensing, handling both syrups and tablets.
- **emergencyNode/**: Monitors patient health metrics to detect emergencies and initiate alerts.
- **motion/**: Controls the robot's movement, enabling navigation within healthcare environments.
- **smartMedicalWatch/**: Interfaces with wearable devices to collect real-time patient health data.
- **patients.db**: A database file that stores patient information securely.
- **ros cheat notes.txt**: A reference file with ROS commands and notes for development purposes.
- **server.py**: Implements the server responsible for managing patient data and coordinating between nodes.
- **static/**: Contains server file mainly the html frontend code.


## Key Features

- **Health Monitoring**: The smart medical watch node collects vital signs and health metrics from patients in real-time.
- **Data Management**: The server node maintains patient records, ensuring data is up-to-date and accessible.
- **Mobility**: The motion node enables the robot to navigate autonomously, assisting patients as needed.
- **Medication Dispensing**: The dispenser node administers medications accurately, managing both syrups and tablets.
- **Emergency Response**: The emergency node detects critical health issues and alerts medical personnel promptly.


>>>>>>> a36247e53591808a1702c34f2d7a13105e79fda4
