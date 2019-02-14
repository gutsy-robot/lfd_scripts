import os
from utils import get_trajectory
import rospy
from baxter_core_msgs.msg import JointCommand, HeadPanCommand
from std_msgs.msg import Bool
from ast import literal_eval

"""

script to read trajectory from MIME txt files and execute on Baxter

"""


def execute_path(data_path):

    left_limb_commander = rospy.Publisher('/robot/limb/left/joint_command', JointCommand, queue_size=10)
    right_limb_commander = rospy.Publisher('/robot/limb/right/joint_command', JointCommand, queue_size=10)
    head_pan_commander = rospy.Publisher('/robot/head/command_head_pan', HeadPanCommand, queue_size=10)
    head_nod_commander = rospy.Publisher('/robot/head/command_head_nod', Bool, queue_size=10)

    rospy.init_node('joint_commander', anonymous=True)
    traj = get_trajectory(data_path)

    for joint_state in traj:

        left_limb_joint_names = []
        left_limb_joint_values = []

        right_limb_joint_names = []
        right_limb_joint_values = []

        head_pan_target = None
        head_nod = False

        joint_state = literal_eval(joint_state)
        for joint_name, state in joint_state.items():

            if joint_name.startswith('left'):
                left_limb_joint_names.append(joint_name)
                left_limb_joint_values.append(state)

            elif joint_name.startswith('right'):
                right_limb_joint_names.append(joint_name)
                right_limb_joint_values.append(state)

            elif joint_name == "head_pan":
                head_pan_target = state

            elif joint_name == "head_nod":
                if abs(state) > 0:
                    head_nod = True
                else:
                    head_nod = False

        left_command = JointCommand()
        left_command.mode = 1
        left_command.names = left_limb_joint_names
        left_command.command = left_limb_joint_values

        right_command = JointCommand()
        right_command.mode = 1
        right_command.names = right_limb_joint_names
        right_command.command = right_limb_joint_values

        head_pan_command = HeadPanCommand()
        head_pan_command.target = head_pan_target
        head_pan_command.speed = 0.1

        head_nod_command = Bool()
        head_nod_command.data = head_nod

        left_limb_commander.publish(left_command)
        right_limb_commander.publish(right_command)

        head_pan_commander.publish(head_pan_command)

        head_nod_commander.publish(head_nod_command)

        rospy.loginfo("State reached...")
        # rospy.sleep(2.0)


execute_path('/home/motion/Desktop/TRINA/1/4317Aug02/joint_angles.txt')














