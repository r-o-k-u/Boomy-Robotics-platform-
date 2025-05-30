# Boomy-Robotics-platform
This project uses ROS 2 (Aurora) as the software framework to control and visualize sensor data from an OpenCV Kinect camera, ADXL accelerometer, and Raspberry Pi Zero W web server.

**System Requirements:**

* ROS 2 (Aurora)
* OpenCV
* Raspberry Pi Zero W
* Arduino Mega
* FlySky FS-iA10B receiver
* ADXL accelerometer
* OpenKinect camera

## system architecture:

**Robot Brain**: The Raspberry Pi Zero W will serve as the robot brain, handling high-level tasks like:
  - Motion planning
  - Sensor integration
  - Communication with other devices (ROS, Bluetooth, etc.)

**Motion Control**: The Arduino Mega will handle motor control, receiving commands from the robot brain and sending control signals to the motor drivers.

**Sensor Integration**: The Kinect sensor and accelerometer will provide data on the robot's movement and orientation, which will be processed by the robot brain.

**ROS Node**: We'll create a ROS node for the robot brain to interact with other devices (e.g., other robots, sensors) using ROS messaging protocols.

+---------------+
|  FlySky FS-iA10B |
|  Receiver        |
+---------------+
           |  Serial Interface
           v
+---------------+
|  Arduino Mega  |
|  (Slave Device) |
+---------------+
           |  Motor Drivers
           v
+---------------+
|  ROS 2 Node for  |
|  Sensor/Actuator  |
|  Control          |
+---------------+
           |  ROS 2 Publisher
           v
+---------------+
|  Raspberry Pi Zero|
|  W (Web Server)    |
+---------------+
           |  ROS 2 Subscriber
           v
+---------------+
|  ROS 2 Node for   |
|  Video Capture and  |
|  Data Display      |
+---------------+

## System Components:

**FlySky FS-iA10B Receiver**: Connect the receiver to the Arduino Mega using a serial interface.
**Arduino Mega (Slave Device)**: Run a ROS 2 node that controls sensors/actuators, motor drivers, and relays.
**ROS 2 Node for Sensor/Actuator Control**: Publish sensor data and control commands from the web server to the ROS 2 node for sensor/actuator control.
**Raspberry Pi Zero W (Web Server)**: Run a Web Server that hosts a user-friendly interface to visualize sensor data, motor control commands, and other relevant data.
**ROS 2 Node for Web Visualization**: Subscribe to data from the ROS 2 node for sensor/actuator control and publish updated web visualization data.
**Zs-X11H BLD-C Motor Drivers**: High-torque motor drivers designed specifically for hoverboard motors. These should provide reliable control over your hoverboard motors.
**Hoverboard Motors**: Excellent choice for creating a hovering or levitation effect on your robot platform.
## Control Modes:

**Transmitter Control**: Connect the FlySky FS-i6 transmitter to the Arduino Mega using a serial interface.
**Autonomous Control**: Run the ROS 2 node for sensor/actuator control without any external input (e.g., via the web server).
**Web Server Control**: Use the Web Server to send motor control commands, sensor data, and other relevant data to the ROS 2 node for sensor/actuator control.
**Bluetooth Control**: Connect a Bluetooth module (e.g., HC-05) to the Raspberry Pi Zero W and use it to send control commands to the ROS 2 node for sensor/actuator control.

## Hardware Layout:


**Base Plate**: Will serve as the foundation of your robot platform. This plate should have mounting holes for:
The Raspberry Pi Zero W (with its power supply)
Arduino Mega
Kinect sensor (with its power supply and cables)
Accelerometer
Motor drivers (Zs-X11H BLD-C)
**Motor Mounting**: Will securely attach the hoverboard motors to the base plate. You can use a combination of screws, brackets, and/or 3D printed parts.
**Power Supply**: Thw power supply system that will meet the requirements of all components:
- Raspberry Pi Zero W: 5V, 2A
- Arduino Mega: 5V, 500mA (assuming it's powered via USB or a separate regulator)
- Kinect sensor: 5V, 1A (assuming it's powered via USB or a separate regulator)
- Accelerometer: typically 3.3V, 100-200mA
- Motor drivers: 12V, 20-30A (depending on the motor specifications)
- Sensors and Cables: Plan for flexible cables to connect sensors and components:
  - Kinect sensor data cables (RGB, depth, etc.)
  - Accelerometer data cable
  - Motor driver control cables
- Shielding: Consider shielding sensitive components like the Arduino Mega and Raspberry Pi Zero W from electromagnetic interference (EMI) and radio-frequency interference (RFI).
Here's a rough sketch of what your hardware layout might look like:

+---------------+
|  Base Plate   |
+---------------+
       |
       |  Power Supply
+---------------+
|  Raspberry Pi  |
|  Zero W        |
+---------------+
       |
       |  Arduino Mega
+---------------+
|  Accelerometer|
+---------------+
       |
       |  Kinect Sensor
+---------------+
|  Motor Drivers |
+---------------+
       |
       |  Motor Mounting System
       |
       v
+---------------+
|  Hoverboard Motors |
+---------------+

**Installation:**

1. Install ROS 2 (Aurora) on your system.
2. Install OpenCV and other required libraries.
3. Set up the Raspberry Pi Zero W as a web server using Node.js or Python.
4. Connect the Arduino Mega to the FlySky FS-iA10B receiver and ADXL accelerometer.
5. Initialize the OpenKinect camera and ROS 2 node for sensor data.

**Usage:**

1. Run the ROS 2 node for sensor data and start publishing sensor data to the web server.
2. Use the web server interface to visualize sensor data, motor control commands, video feed from the OpenCV Kinect camera, and data from the ADXL accelerometer.

**Example Use Cases:**

* Robot navigation and mapping
* Object recognition and tracking
* Human-robot interaction

**Troubleshooting:**

* Check the serial port and baudrate configuration for the FlySky FS-iA10B receiver.
* Verify that the OpenKinect camera is properly connected to the Arduino Mega.
* Ensure that the ROS 2 node for sensor data is running correctly.

**License:**

This project is licensed under the [Your License] license. See the LICENSE file for more information.

**Contributors:**

* [Your Name]
* [Other contributors]

**Acknowledgments:**

We would like to acknowledge the following organizations and individuals for their contributions to this project:

* OpenCV
* ROS 2 (Aurora)
* Raspberry Pi Foundation

I hope this helps! Let me know if you have any questions or need further assistance.
