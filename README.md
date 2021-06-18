# sport_sole

## **sport_sole**
This is the name of the package  
Contains all programs related to the shoe and sole project

## **launch**
test1.launch is the launch file that opens  
* sport_sole_publisher.cpp
* rviz (to visualize acceleration and orientation of shoes with arrow)
* rqt_plot (graph of shoe's acceleration data, which shows orientation)  
> $ roslaunch sport_sole test1.launch

## **msg**
SportSole.msg contains custom message types for data collected in the data packets
* acceleration
* quaternion
* pressure  

Accessed in sport_sole_publisher.cpp  

## **rviz**
Don't open accel_orient.rviz from VS Code  
Instead, this automatically opens when the launch file is activated   
accel_orient.rviz visualizes acceleration and orientation of shoes with arrows

## **src**
sport_sole_publisher.cpp is the main code for this project  
Purpose: ??  
Type name: sport_sole_publisher

## **CMakeLists.txt**
Lists the depenencies that allow sport_sole_publisher.cpp to run  

## **package.xml**
Similar purpose as CMakeLists.txt  


# Configurations
The following configurations assume that you use a TP-Link router.

1. The router IP address must be `192.168.1.254`. This can be set in "Advanced" -> "Network" -> "IP Address".
1. The address of the host PC must be `192.168.1.100`. This can be set in "Advanced" -> "DHCP" -> "Address Reservation".
1. The router SSID must be named "SportShoe". This can be set in "Advanced" -> "Wireless 2.4GHz" -> Wireless Settings" -> "Wireless Network Name"
1. The network traffic can be monitored in "Advanced" -> "Wireless 2.4GHz" -> "Wireless Statistics".
1 The IP address of "PD shoe left" and "PD shoe right" should be `192.168.1.11` and `192.168.1.12`, respectively.

## Reset the SSID of XBee module
1. Flash the teensy with firmware "Xbee_AT_transparent.ino"
1. Send "+++" with no carriage return via serial. Teensy should respond with "Ok".
1. Immediately after the previous step, send "atid" with carriage return. Teensy should respond with the current SSID.
1. Immediately after the previous step, send "atid newssid" with carriage return. Teensy should respond with the new SSID "newssid".
1. Immediately after the previous step, send "atwr" with carriage return to write the change to XBee.
* Note that timeout may occur if you wait too long after each step.

## debug udp packet
sudo tcpdump -i enp60s0 -x udp