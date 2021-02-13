#!/usr/bin/env python
import rospy
from avatar_msgs.msg import JointStateExpanded
from std_msgs.msg import String


def callback(data):
    # rospy.loginfo("message", data)
    print(data.name[0], type(data.name[0]))


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node("listener", anonymous=True)

    rospy.Subscriber("chatter", JointStateExpanded, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == "__main__":
    listener()