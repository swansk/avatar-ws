#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64
import odrive
from odrive.enums import *

def pos_publisher():
    my_odrive = odrive.find_any()

    if my_odrive == None:
        print("Could not find ODrive, exiting...")
        return -1

    pub = rospy.Publisher('/avatar/gearShaft2_position_controller/command', Float64, queue_size=10)
    pub2 = rospy.Publisher('/avatar/mainAxle_position_controller/command', Float64, queue_size=10)
    rospy.init_node('pos_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        pos_estimate = my_odrive.axis0.encoder.pos_estimate
        angle_estimate_rad = (pos_estimate*-6.28) - 0.4
        rospy.loginfo(my_odrive.axis0.encoder.pos_estimate)

        pos_estimate2 = my_odrive.axis1.encoder.pos_estimate
        angle_estimate_rad2 = (pos_estimate2*-6.28) + 0.7

        pub2.publish(angle_estimate_rad2)
        # rospy.loginfo(my_odrive.axis1.encoder.pos_estimate)
        pub.publish(angle_estimate_rad)
        rate.sleep()

if __name__ == '__main__':
    try:
        pos_publisher()
    except rospy.ROSInterruptException:
        pass