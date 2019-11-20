# Open Mobile Manipulator Project

![Mobile_Manipulator](Mobile_Manipulator.jpg)

### Description

This project is meant to help you with creating your own **cheap education mobile manipulator**, 
The robot will primarily consist of a robot base (from wood) see assembly, 4 DC **geared** motors with quadrature encoders (for pid control and odometry), 1 arduino due for the hardware communication and reading the encoders, 1 Jetson nano to run ROS on, 1 2D lidar, 1 kinect camera and a robotic arm.

This can help you to learn Move-it, Navigation, Rtabmap, Amcl on a real robot and hardware interface, I will try to make some Gazebo demos for all of that.

### List of materials
[Geared dc motor x4](https://www.robotshop.com/eu/en/lynxmotion-12vdc-200rpm-078kg-cm-ghm-16-w--rear-shaft.html)
[quadrature encoder x4](https://www.robotshop.com/eu/en/lynxmotion-quadrature-motor-encoder-v2-cable.html)
[arm 6 dof amazon uk](https://www.amazon.co.uk/gp/product/B07VYHFL1V/ref=ppx_yo_dt_b_asin_title_o01_s00?ie=UTF8&psc=1)
Jetson Nano ~ 100
Arduino Due ~ 30
L298N motor driver
2D ydlidar ~ 100
1 lipo 5000 mah battery ~ 50
1 usb wifi anttena
1 16-bit Adafruit servo shield I2c
**The total cost should be approximately 500 - 700 euro**
4 **steel** brackets for the dc motors

### Assembly
The easiest way to build the robot base is from wood, cut 1 piece 20x30 and another 20x35 (0.8cm thick hard wood), then make 4 8mm holes in the woods and place 8mm threaded rods with nuts so you can have 2 levels, in the bottom level place all the electronics (lidar elevated) and in the top level place the arm jetson nano and kinect camera (which should be elevated somehow as the piture shows)

### For real robot run the command

roslaunch lynxbot_bringup lynxbot_bringup.launch

roslaunch lynxbot_bringup moveit.launch
roslaunch lynxbot_bringup gmapping.launch
roslaunch lynxbot_bringup teleop.launch

### For simulation

roslaunch lynxbot_simulation sim_bringup.launch

roslaunch lynxbot_bringup moveit.launch
roslaunch lynxbot_bringup gmapping.launch
roslaunch lynxbot_bringup teleop.launch

