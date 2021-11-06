# GY-US42-Sonar
A ROS2 Node to run on a Raspberry Pi.  Reads the GY-US42 Sonar via I2C and publishes a sensor_msgs/msg/Range topic.  

Development environment specifics:
Raspberry Pi 3
Ubuntu 20.04.2 LTS

# Status
The code runs successfully, reading the sonar and publishing the expected messages.  Deemed production ready.

## Considerations
I cannot find an official data sheet for this sensor.  All the references state that the device works like the Maxbotix MaxSonar.  I detected the i2c address (0x70) and applied the Maxbotix logic.  When I read two bytes for the range the high byte is allways 128 when the range is less than 256 cm.  The moment a second byte is needed to represent distance, the high byte increases to 129 and 130 respectively.  I argue that the correct range should be the low_byte + (high_byte - 128) * 100.  This is rather unique, but seems to work.  If anyone has alternative wisdom (or the data sheet) please advise.  I also found the maximum range for the node to be 400 cm.  Annything further returns 402.  Minimim distance measured is abour 20cm. 

## Improvements
Node can be improved by adding parameters for the i2c sensor address (0x70)

# Installation
Wire the sensor as per manufacturers specifications.  Enable the i2c port on the Raspberry Pi.  A good exmple can be found at [Sparkfun Raspberry Pi SPI and I2C Tutorial](https://learn.sparkfun.com/tutorials/saprberry-pi-spi-and-i2c-tutorial/all).  Ensure that the user that will run the ROS node is in the correct group.
```
sudo usermod -aG i2c ubuntu
```

Clone this repo into the src directory of your ROS2 workspace. See the [ros2 tutorial](https://docs.ros.org/en/foxy/Tutorials/Workspace/Creating-A-Workspace.html) on how to create a workspace.
```
git clone https://github.com/slaghuis/GY-US42-Sonar.git
```
Back in the root of your ROS workspace, build and install the package.  
```
colcon build --packages-select sonar
. install/setup.bash
```
Run the package
```
ros2 run sonar sonar_node
```
See the output in a seperate terminal
```
ros2 topic echo vl53l1x/range
```
