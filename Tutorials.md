# Tutorials
This file contains a bunch of tutorial for the car or ROS2, it's a good idea to read it once and go there if you have doubt on something

- [Tutorials](#tutorials)
    - [Create a dynamic connection for USB devices](#create-a-dynamic-connection-for-usb-devices)
      - [Find idVendor and idProduct of a device](#find-idvendor-and-idproduct-of-a-device)
      - [Create a udev-rule file](#create-a-udev-rule-file)
    - [Create a Package](#create-a-package)
      - [Name and use of the package](#name-and-use-of-the-package)
      - [Creation of a package](#creation-of-a-package)
      - [Package from external source](#package-from-external-source)
      - [Node](#node)
        - [Publisher/Subscriber](#publishersubscriber)
        - [Messages/Services](#messagesservices)
        - [Launch files](#launch-files)
      - [Setup and Package file](#setup-and-package-file)
      - [Python package](#python-package)
      - [At the end](#at-the-end)
    - [Control the car from your PC](#control-the-car-from-your-pc)
      - [Troubleshooting](#troubleshooting)
    - [Quick Lesson and Tutorials on log in ROS2](#quick-lesson-and-tutorials-on-log-in-ros2)
      - [Severity level](#severity-level)
      - [On your code](#on-your-code)
      - [Changing the Severity level of your node](#changing-the-severity-level-of-your-node)


---
### Create a dynamic connection for USB devices
Our car use (for now) two USB connection, one for the Lidar and another one for the Dynamixel motor (via U2D2). Normally, when a device is connected to the RPi5, a serial port is given for it. It will be /dev/ttyUSB* (with *, a given number). This port is needed to communicate with our devices in our codes. The problem is that at every reboot of our computer, the ports can changed and so we always need to chang eit in our codes. To counter this we will create a dynamical port for our devices based on their identity.
#### Find idVendor and idProduct of a device
You will first need to have the idVendor and idProduct of your device, to do this you will type on the terminal :
```shell
lsusb
```
this will give you a list of device like :
```yaml
Bus 002 Device 003: ID 1234:5678 Manufacturer_Name Product_Name
```
Find you device and note the number after the 'ID' term.
#### Create a udev-rule file
A udev-rule file is a file with some commands that will be launch at every boot. To create it please type on a terminal :
```shell
sudo nano /etc/udev/rules.d/99-usb-device.rules
```
Then in the file write :
```shell
SUBSYSTEM=="tty", ATTRS{idVendor}=="1234", ATTRS{idProduct}=="5678", SYMLINK+="ttyDEVICE", MODE="0777"
```
You can quit the file with CTRL+O and CTRL+X to save and quit it. The reboot the RPi5.
**We did this for the Lidar (ttyLIDAR) and the Dynamixel (ttyU2D2)**

### Create a Package
If you want to transform a package from ROS to ROS2 or to add your own package, please follow these instructions correctly to avoid losing time and to keep a clean and clear environment. All these informations are mainly inspired by the official [tutorials](https://docs.ros.org/en/jazzy/Tutorials.html) of ROS2 (here jazzy distribution), that you followed at the beginning of the class.
#### Name and use of the package
It is important to have a src folder easy to understand. We don't want for you and the futur group to loose a week or two to understand the workspace. So please read this steps if you want to create a package. 
Firstly be sure that the creation of a package is necessary. By exemple, if you want another package which will work on the STM32, just create your node on the bolide_stm32 package or modify an node already existing. The good practice is a package for a device. Here we have a package for all the stuff with the STM32, one for the lidar and another one for the Dynamixel.
Another good practice is to have packages for low-level code (getting and setting data directly from sensors, actuators) and packages for high-level code (controller, big algorithm, slam, ...). With this we'll have a beautiful and nice src folder. 
If you're not sure, ask teachers or older groups (my mail can be find in setup file of a package)
#### Creation of a package
To create a package you need to be in the src folder (in this repository for a low-level node or in the other for a high-level node) of your workspace on a terminal and use the command line :
```shell
ros2 pkg create —build-type ament_python —license Apache-2.0 package_name
```
After the creation done you will see your folder and all the basic files needed.
#### Package from external source
If you want to use a package already made by someone for a sensor (e.g: the lidar), then just proceed to the instructions given by the creator (generally on the README file of the github page) and make sure to be at the good place.
#### Node
In your node be as clear as possible. We're not in the 80s and we got enough technology to help us coding that you can be clear and precise. Don't name your value like 'a' or 'br', it is not easy to understand their meaning so name them like 'acceleration' or 'baudrate'. Like this we can understand in one read what represent the value and we don't need to add comments everywhere. It is also recommended to describe your file, class, function, ..., put your name and surname at the beginning to know who contact if we need it. By describing this element we can, again, know easily what does the module without reading 3 hours by files.
##### Publisher/Subscriber
For the topic name use something understandable (like '/lidar_data') or some basic naming in ROS2 (like '/cmd_vel'). Put a '/' at the beginning of the topic name.
##### Messages/Services
For messages and services that you create, put them all in one and only one package. Here this package is [bolide_interfaces](./bolide_interfaces/).
##### Launch files
If you want to use launch files create in your package folder a 'launch' folder where you will put your launch codes. Name your file like 'name_of_the_launch.launch.py'. If a package have a launch file, never forget to add in its 'setup.py' file this :
```python
data_file=[
    ...
    (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', 'launch.[pxy][yma]'))),
]
```
#### Setup and Package file
In your 'setup.py' file you need to add all your nodes name and source like this :
```python
entry_points={
        'console_scripts': [
            'node_name = pkg_name.node_file_name:main',
        ],
    },
```
In your 'package.xml' you need to add all the dependencies of your package like this (e.g):
```xml
<depend>rclpy</depend>
<depend>another_pkg</depend>
```
If a package is dependant of another package in your workspace you will need to build firstly the dependence.
#### Python package
In your code you will surely use some external Python packages like 'spidev', but recently Ubuntu rules about global environment changed and we can't just install with pip the package. To be able to use external package we use a virtual environment. The virtual environment is in the venv folder on the RPi5. To activate it use this command line in the workspace:
```shell
source /venv/bin/activate
```
All command usually working in the terminal still work here. If you want to install some packages use 'pip install package_name'. With this you can use any package in python for your ROS2 node. You will need to go in the 'setup.cfg' file of your package and add this :
```pkg
[build_scripts]
executable = /usr/bin/env python3
```
I don't know why, but if you use some external Python packages with the virtual environment, --symlink-install can not working well so you'll need to colcon build without it. TODO - check this to solve the problem
#### At the end
At the end our src folder need to look like this:

- bolides_interfaces/
- bolide_direction/
- bolide_.../
- sllidar_ros2/
- external_pkg2/
- ...
---

### Control the car from your PC

Follow these steps to control the car from your PC:

1. **Connect to the Same Network (Or any other VMs just make sure that the 4th step is made)**  
   - Connect both your Raspberry Pi 5 (Rpi5) and your PC (Ubuntu24.04 or VMware) to the same hotspot (e.g., your 4G/5G or Wi-Fi).  
   - For now, the default 4G connection on the Rpi5 is: `iPhone de Babou (2)`. A more generic Wi-Fi will be added later.

2. **Test the Connection**  
   - On your RPi5, run the following command to get its IP address:  
     ```shell
     hostname -I
     ```
     Note the first IP address (`<IP_ROBOT>`).  
   - On your PC, test the connection by pinging the RPi5:  
     ```shell
     ping <IP_ROBOT>
     ```
     You should see output similar to this:  
     ```
     PING 172.20.10.9 (172.20.10.9) 56(84) bytes of data.
     64 bytes from 172.20.10.9: icmp_seq=1 ttl=64 time=1025 ms
     64 bytes from 172.20.10.9: icmp_seq=2 ttl=64 time=7.18 ms
     64 bytes from 172.20.10.9: icmp_seq=3 ttl=64 time=146 ms
     64 bytes from 172.20.10.9: icmp_seq=4 ttl=64 time=10.8 ms
     64 bytes from 172.20.10.9: icmp_seq=5 ttl=64 time=10.8 ms
     ```
   - Repeat the same steps in reverse:  
     On your PC, run `hostname -I` and ping the PC's IP address from the RPi5.

3. **Set the ROS Domain ID**  
   - On your RPi5, check the ROS Domain ID:  
     ```shell
     echo $ROS_DOMAIN_ID
     ```
     By default, it should be `10`.  
   - On your PC (VMware), set the same ROS Domain ID:  
     ```shell
     export ROS_DOMAIN_ID=10
     ```

4. **Launch the Teleoperation Node**  
   - On your PC, launch the teleoperation node from the `planning_bolide` package:  
     ```shell
     ros2 run planning_bolide teleop_node
     ```

#### Troubleshooting

If you encounter any issues, please double-check the steps above or ask for assistance. Some key steps might have been overlooked.

---
### Quick Lesson and Tutorials on log in ROS2
Logging is a subsystem method in ROS2 which permits to send log message to 3 different targets :
- the log files on your computer (normally at ~/.ROS folder)
- to the /rosout topic on the ROS2 Network
- on your terminal when at the correct severity level

#### Severity level
there is 5 severity levels for log messages :
- DEBUG (10)
- INFO (20)
- WARN (30)
- ERROR (40)
- FATAL (50)

Normally your severity level is put on INFO level. It means that only INFO message and higher (WARN, ERROR and FATAL) will be see on your terminal.

#### On your code
When creating a node, ROS2 will automatically attach a logging system to it, with the name of your node, set at INFO severity level. To use log message you will need to type in your code (if you're in a Node Class):
```python
self.get_logger().info("message")
```
This will sent an INFO severity level message. Use .debug( ), .warn( ), .error( ), .fatal( ) for other severity levels.

A good practice to have is to use the correct severity level for your message. An INFO level when it is a simple message, a WARN when you want to warn about something, ERROR when there is a error, ...

With your message, you can also indicate if you want this log to be print only once, every x seconds, or every time except the first time with this line :
```python
self.get_logger().info("message", once=True)
```
```python
self.get_logger().info("message", throttle_duration_sec=x_seconds)
```
```python
self.get_logger().info("message", skip_first=True)
```

#### Changing the Severity level of your node
As you saw before, you can't to print a DEBUG log message on your terminal because the severity level is set to INFO. To change this you just need to type in your code:
```python
rclpy.logging().set_logger_level('name_of_the_node', 10)
```

