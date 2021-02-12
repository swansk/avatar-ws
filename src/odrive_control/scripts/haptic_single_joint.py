#!/usr/bin/env python
# license removed for brevity
import collections
import math
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from control_msgs.msg import JointControllerState
import odrive
from odrive.enums import *

AXIS_STATE_IDLE = 1
AXIS_STATE_CLOSED_LOOP_CONTROL = 8
CONTROL_MODE_VOLTAGE_CONTROL = 0
CONTROL_MODE_TORQUE_CONTROL = 1
CONTROL_MODE_VELOCITY_CONTROL = 2
CONTROL_MODE_POSITION_CONTROL = 3

ENVIRONMENTAL_I = 0.5


class ApeHaptics:
    def __init__(self):
        self.actual_pos_sim = 0
        self.actual_vel_sim = 0
        self.cmd_pos_sim = 0
        self.sim_cmd_position_pub = rospy.Publisher(
            "/avatar/mainAxle_position_controller/command",
            Float64,
            queue_size=10,
        )

        rospy.Subscriber(
            "/avatar/joint_states",
            JointState,
            self.update_actual_pos_sim,
        )
        rospy.Subscriber(
            "/avatar/mainAxle_position_controller/state",
            JointControllerState,
            self.update_cmd_pos_sim,
        )
        rospy.init_node("ape_haptics", anonymous=True)

        self.odrive = odrive.find_any()

        if self.odrive == None:
            raise RuntimeError("Could not find ODrive")

    def update_actual_pos_sim(self, joint_states):
        position_rad = joint_states.position[2]
        velocity_m = joint_states.velocity[2]
        self.actual_pos_sim = position_rad
        self.actual_vel_sim = velocity_m

    def update_cmd_pos_sim(self, joint_controller_state):
        self.cmd_pos_sim = joint_controller_state.set_point

    def run(self):
        r = rospy.Rate(100)
        self.odrive.axis0.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
        while not rospy.is_shutdown():
            self.sim_cmd_position_pub.publish(self.actual_pos_sim)
            odrive_pos_circular = self.odrive.axis0.encoder.pos_circular

            sim_cmd_position = odrive_pos_circular * 6.28 - 0.5
            pos_diff = self.actual_pos_sim - self.cmd_pos_sim
            if abs(pos_diff) > 0.15:
                self.odrive.axis0.controller.input_torque = pos_diff * ENVIRONMENTAL_I
                self.odrive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            else:
                self.odrive.axis0.requested_state = AXIS_STATE_IDLE

            self.sim_cmd_position_pub.publish(sim_cmd_position)
            r.sleep()


if __name__ == "__main__":
    ape_haptics = ApeHaptics()
    try:
        ape_haptics.run()
    except rospy.ROSInterruptException:
        pass