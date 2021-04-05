#!/usr/bin/env python
import rospy
import math
import numpy as np
from numpy import cos, sin, array
from avatar_msgs.msg import JointStateExpanded
from std_msgs.msg import String

# Globals and constants
pub = rospy.Publisher("jacobian", JointStateExpanded, queue_size=10)
rospy.init_node("angle talker", anonymous=True)
rate = rospy.Rate(1)  # 1hz

pi = 3.14159

# Set up link lengths (arbitrary at the moment)
l0 = 131.9 / 30
l1 = 15.8 / 30
l2 = 130.9 / 30
l3 = 176.5 / 30
l4 = 300 / 30
l5 = 310 / 30

# Motor revolution "theta 1" shortened to t1
# Ensure radians used

rad90 = 90 / 360 * 2 * pi
rad45 = 45 / 360 * 2 * pi


def HTransform(n):
    t = n[0]  # c(tn)       -s(tn)*c(an)    s(tn)*s(an)     rn*c(tn)
    a = n[1]  # s(tn)       c(tn)*c(an)     -c(tn)*s(an)    rn*s(tn)
    r = n[2]  # 0           s(an)           c(an)           dn
    d = n[3]  # 0           0               0               1

    H = array(
        [
            [cos(t), -sin(t) * cos(a), sin(t) * sin(a), r * cos(t)],
            [sin(t), cos(t) * cos(a), -cos(t) * sin(a), r * sin(t)],
            [0, sin(a), cos(a), d],
            [0, 0, 0, 1],
        ]
    )
    return H


def callback(data):
    # rospy.loginfo("message", data)

    t0 = data.position[0]  # hopefully in radians
    t1 = data.position[1]
    t2 = data.position[2]
    t3 = data.position[3]

    DH7 = np.array(
        [
            [t0, 0, -l0, 0],
            [rad90, rad45, 0, l1],
            [t1, 0, -l2, 0],
            [rad90, -rad90, 0, l3],
            [rad90 + t2, 0, -l4, 0],
            [t3, 0, -l5, 0],
        ]
    )

    # Homogenous Transform Matrices
    H01 = HTransform(DH7[0])
    H12 = HTransform(DH7[1])
    H23 = HTransform(DH7[2])
    H34 = HTransform(DH7[3])
    H45 = HTransform(DH7[4])
    H56 = HTransform(DH7[5])

    H02 = np.matmul(H01, H12)
    H03 = np.matmul(H02, H23)
    H04 = np.matmul(H03, H34)
    H05 = np.matmul(H04, H45)
    H06 = np.matmul(H05, H56)

    # Displacement Matrices

    D01 = H01[0:3, 3:4]
    D02 = H02[0:3, 3:4]
    D03 = H03[0:3, 3:4]
    D04 = H04[0:3, 3:4]
    D05 = H05[0:3, 3:4]
    D06 = H06[0:3, 3:4]

    # Unused????
    # D12 = H12[0:3, 3:4]
    # D23 = H23[0:3, 3:4]
    # D34 = H34[0:3, 3:4]
    # D45 = H45[0:3, 3:4]
    # D56 = H56[0:3, 3:4]

    # now publish the arm parameters
    arm_params = JointStateExpanded()
    arm_params.name = [
        "joint0",
        "joint1",
        "joint2",
        "joint3",
        "joint4",
        "joint5",
        "joint6",
    ]
    # we have what the position in 3D space of each joint should be below, but not
    # what the angles of each link would have to be to achieve this
    # (JointStateExpanded.position is angles I think)
    joint_positions = [
        [0, 0, 0],
        [D01[0], D01[1], D01[2]],
        [D02[0], D02[1], D02[2]],
        [D03[0], D03[1], D03[2]],
        [D04[0], D04[1], D04[2]],
        [D05[0], D05[1], D05[2]],
        [D06[0], D06[1], D06[2]],
    ]
    arm_params.position = []
    pub.publish()


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