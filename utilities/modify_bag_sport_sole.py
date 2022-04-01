#!/usr/bin/python

import rosbag
import rospy
import rospkg
from sport_sole.msg import SportSole

rp = rospkg.RosPack()
path = rp.get_path('gait_training_robot') + '/bags/'
in_bag_path = path + 'test010c.bag'
out_bag_path = path + 'test010b.bag'
print('Modifying certain messages in a bag file')
print('Input bag: ' + in_bag_path)
print('Output bag: ' + out_bag_path)


with rosbag.Bag(in_bag_path, 'r') as inbag, rosbag.Bag(out_bag_path, 'w') as outbag:
    count = [0, 0]
    for topic, msg, ts in inbag.read_messages():
        if topic == '/sport_sole_publisher/sport_sole':
            msg.header.frame_id = 'camera_base'
            count[0] += 1
        outbag.write(topic, msg, ts)
        
    
    
print('Migration completed: ' + str(count[0]) + ' messages have been modified. '
    + str(count[1]) + ' messages have been deleted.')