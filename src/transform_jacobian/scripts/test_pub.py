#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from avatar_msgs.msg import JointStateExpanded

def talker():
    pub = rospy.Publisher('chatter', JointStateExpanded, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    
    hello_str = JointStateExpanded()
    hello_str.name = ['Shneep Shnop', 'Bababooey', 'Pterodactyl', 'Karl']
    hello_str.position = [1.51, 2.21, 3.81, 4.11]
    hello_str.velocity = [1.52, 2.87, 3.22, 4.94]
    hello_str.effort = [10.44, 23.66, 35.76, 48.92]
    hello_str.current = [1, 2, 3, 4]

    while not rospy.is_shutdown():
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass