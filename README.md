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

