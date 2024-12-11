#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
import numpy as np
from numpy import linalg
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import String
from intera_interface import gripper as robot_gripper
from intera_interface import Limb
import sys
import tf.transformations as tf


FOOD_LOC = (0.691, -0.1, 0.383)
UTENSIL_LOC = (0.691, -0.324, 0.383)
class IKExample():

    def __init__(self):

        self.goal_point_sub = rospy.Subscriber("/goal_point", Point, self.goal_img_callback) 
        self.obj_type_sub = rospy.Subscriber("/obj_type", String, self.obj_type_callback) 
    

    def goal_img_callback(self, msg):
        self.goal_x = msg.x 
        self.goal_y = msg.y 
        self.goal_z = msg.z

    def obj_type_callback(self, msg):
        self.obj_type = msg.data



def main():
    # Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
    rospy.init_node('service_query')
    # Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    # self.goal_point_sub = rospy.Subscriber("/goal_point", Point, None)
    # Set up the right gripper
    right_gripper = robot_gripper.Gripper('right_gripper')
    limb = Limb("right")

    current_angles = limb.joint_angles()
    print(current_angles)
    ik_class = IKExample()

    while not rospy.is_shutdown():

        # Open the right gripper
        print('Opening...')
        right_gripper.open()
        rospy.sleep(1.0)
        print('Done!')

        #                 # Python's syntax for a main() method# Construct the request
        # request = GetPositionIKRequest()
        # request.ik_request.group_name = "right_arm"

        # input('Press [ Enter ] for first movement: ')

        # # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
        # # link = "stp_022312TP99620_tip_1"
        # link = "right_gripper_tip"

        # request.ik_request.ik_link_name = link
        # # request.ik_request.attempts = 20
        # request.ik_request.pose_stamped.header.frame_id = "base"
        
        # # Set the desired orientation for the end effector HERE
        # request.ik_request.pose_stamped.pose.position.x = type_loc[0]
        # request.ik_request.pose_stamped.pose.position.y = type_loc[1] 
        # request.ik_request.pose_stamped.pose.position.z = type_loc[2] 
        # # request.ik_request.pose_stamped.pose.position.x = 0.687 
        # # request.ik_request.pose_stamped.pose.position.y = 0.159
        # # request.ik_request.pose_stamped.pose.position.z = 0.383    
        # request.ik_request.pose_stamped.pose.orientation.x = 0.0
        # request.ik_request.pose_stamped.pose.orientation.y = 1.0
        # request.ik_request.pose_stamped.pose.orientation.z = 0.0
        # request.ik_request.pose_stamped.pose.orientation.w = 0
        
        # try:
        #     # Send the request to the service
        #     response = compute_ik(request)
            
        #     # Print the response HERE
        #     print(response)
        #     group = MoveGroupCommander("right_arm")

        #     # Setting position and orientation target
        #     group.set_pose_target(request.ik_request.pose_stamped)

        #     # TRY THIS
        #     # Setting just the position without specifying the orientation
        #     # group.set_position_target([ik_class.goal_x, ik_class.goal_y, ik_class.goal_z])

        #     # Plan IK
        #     plan = group.plan()
        #     user_input = input("Enter 'y' if the trajectory looks safe on RVIZ")
            
        #     # Execute IK if safe
        #     if user_input == 'y':
        #         group.execute(plan[1])
            
        # except rospy.ServiceException as e:
        #     print("Service call failed: %s"%e)
        
        
        
        # Construct the request
        request = GetPositionIKRequest()
        request.ik_request.group_name = "right_arm"

        input('Press [ Enter ] for first movement: ')

        #8.5 x
        # y = 0
        # z=-3
        #9 deg


        # new calibration 
        # 17.5 x
        # 0 y
        # 11.75 z

        # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
        # link = "stp_022312TP99620_tip_1"
        link = "right_gripper_tip"

        request.ik_request.ik_link_name = link
        # request.ik_request.attempts = 20
        request.ik_request.pose_stamped.header.frame_id = "base"
        
        # Set the desired orientation for the end effector HERE
        request.ik_request.pose_stamped.pose.position.x = ik_class.goal_x
        request.ik_request.pose_stamped.pose.position.y = ik_class.goal_y
        request.ik_request.pose_stamped.pose.position.z = ik_class.goal_z 
        # request.ik_request.pose_stamped.pose.position.x = 0.671
        # request.ik_request.pose_stamped.pose.position.y = 0.214
        # request.ik_request.pose_stamped.pose.position.z = -0.118      
        request.ik_request.pose_stamped.pose.orientation.x = 0.0
        request.ik_request.pose_stamped.pose.orientation.y = 1.0
        request.ik_request.pose_stamped.pose.orientation.z = 0.0
        request.ik_request.pose_stamped.pose.orientation.w = 0.0
        
        try:
            # Send the request to the service
            response = compute_ik(request)
            
            # Print the response HERE
            print(response)
            group = MoveGroupCommander("right_arm")

            # Setting position and orientation target
            group.set_pose_target(request.ik_request.pose_stamped)

            # TRY THIS
            # Setting just the position without specifying the orientation
            # group.set_position_target([ik_class.goal_x, ik_class.goal_y, ik_class.goal_z])

            # Plan IK
            plan = group.plan()
            user_input = input("Enter 'y' if the trajectory looks safe on RVIZ")
            
            # Execute IK if safe
            if user_input == 'y':
                group.execute(plan[1])
            
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        
        # Close the right gripper
        print('Closing...')
        right_gripper.close()
        rospy.sleep(2.0)

                # Construct the request
        request = GetPositionIKRequest()
        request.ik_request.group_name = "right_arm"

        input('Press [ Enter ] for first movement: ')

        #8.5 x
        # y = 0
        # z=-3
        #9 deg


        # new calibration 
        # 17.5 x
        # 0 y
        # 11.75 z

        # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
        # link = "stp_022312TP99620_tip_1"
        link = "right_gripper_tip"

        request.ik_request.ik_link_name = link
        # request.ik_request.attempts = 20
        request.ik_request.pose_stamped.header.frame_id = "base"
        
        # Set the desired orientation for the end effector HERE
        # request.ik_request.pose_stamped.pose.position.x = 0.671
        # request.ik_request.pose_stamped.pose.position.y = 0.214
        # request.ik_request.pose_stamped.pose.position.z = -0.118   
        request.ik_request.pose_stamped.pose.position.x = 0.691 
        request.ik_request.pose_stamped.pose.position.y = 0.159
        request.ik_request.pose_stamped.pose.position.z = 0.383    
        request.ik_request.pose_stamped.pose.orientation.x = 0.0
        request.ik_request.pose_stamped.pose.orientation.y = 1.0
        request.ik_request.pose_stamped.pose.orientation.z = 0.0
        request.ik_request.pose_stamped.pose.orientation.w = 0.0
        
        try:
            # Send the request to the service
            response = compute_ik(request)
            
            # Print the response HERE
            print(response)
            group = MoveGroupCommander("right_arm")

            # Setting position and orientation target
            group.set_pose_target(request.ik_request.pose_stamped)

            # TRY THIS
            # Setting just the position without specifying the orientation
            # group.set_position_target([ik_class.goal_x, ik_class.goal_y, ik_class.goal_z])

            # Plan IK
            plan = group.plan()
            user_input = input("Enter 'y' if the trajectory looks safe on RVIZ")
            
            # Execute IK if safe
            if user_input == 'y':
                group.execute(plan[1])
            
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        
        # Close the right gripper
        print('Closing...')
        right_gripper.close()
        rospy.sleep(2.0)


        # Construct the request
        request = GetPositionIKRequest()
        request.ik_request.group_name = "right_arm"

        input('Press [ Enter ] for second movement: ')

        # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
        # link = "stp_022312TP99620_tip_1"
        link = "right_gripper_tip"

        request.ik_request.ik_link_name = link
        # request.ik_request.attempts = 20
        request.ik_request.pose_stamped.header.frame_id = "base"
        

        if ik_class.obj_type == 'food':
            type_loc = FOOD_LOC 
        else:
            type_loc = UTENSIL_LOC
        # Set the desired orientation for the end effector HERE
        request.ik_request.pose_stamped.pose.position.x = type_loc[0]
        request.ik_request.pose_stamped.pose.position.y = type_loc[1]
        request.ik_request.pose_stamped.pose.position.z = type_loc[2]
        request.ik_request.pose_stamped.pose.orientation.x = 0.0
        request.ik_request.pose_stamped.pose.orientation.y = 1.0
        request.ik_request.pose_stamped.pose.orientation.z = 0.0
        request.ik_request.pose_stamped.pose.orientation.w = 0.0
        
        try:
            # Send the request to the service
            response = compute_ik(request)
            
            # Print the response HERE
            print(response)
            group = MoveGroupCommander("right_arm")

            # Setting position and orientation target
            group.set_pose_target(request.ik_request.pose_stamped)

            # TRY THIS
            # Setting just the position without specifying the orientation
            # group.set_position_target([ik_class.goal_x, ik_class.goal_y, ik_class.goal_z])

            # Plan IK
            plan = group.plan()
            user_input = input("Enter 'y' if the trajectory looks safe on RVIZ")
            
            # Execute IK if safe
            if user_input == 'y':
                group.execute(plan[1])
            
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


        
        # # Python's syntax for a main() method# Construct the request
        # request = GetPositionIKRequest()
        # request.ik_request.group_name = "right_arm"

        # input('Press [ Enter ] for third movement: ')

        # # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
        # # link = "stp_022312TP99620_tip_1"
        # link = "right_gripper_tip"

        # request.ik_request.ik_link_name = link
        # # request.ik_request.attempts = 20
        # request.ik_request.pose_stamped.header.frame_id = "base"
        
        # # Set the desired orientation for the end effector HERE
        # request.ik_request.pose_stamped.pose.position.x = type_loc[0]
        # request.ik_request.pose_stamped.pose.position.y = type_loc[1] 
        # request.ik_request.pose_stamped.pose.position.z = type_loc[2] 
        # # request.ik_request.pose_stamped.pose.position.x = 0.747 
        # # request.ik_request.pose_stamped.pose.position.y = 0.971
        # # request.ik_request.pose_stamped.pose.position.z = 0.781    
        # request.ik_request.pose_stamped.pose.orientation.x = 0.0
        # request.ik_request.pose_stampto take commands for planning group right_aed.pose.orientation.y = 0.0
        # request.ik_request.pose_stamped.pose.orientation.z = 1.0
        # request.ik_request.pose_stamped.pose.orientation.w = 0
        
        # try:
        #     # Send the request to the service
        #     response = compute_ik(request)
            
        #     # Print the response HERE
        #     print(response)
        #     group = MoveGroupCommander("right_arm")

        #     # Setting position and orientation target
        #     group.set_pose_target(request.ik_request.pose_stamped)

        #     # TRY THIS
        #     # Setting just the position without specifying the orientation
        #     # group.set_position_target([ik_class.goal_x, ik_class.goal_y, ik_class.goal_z])

        #     # Plan IK
        #     plan = group.plan()
        #     user_input = input("Enter 'y' if the trajectory looks safe on RVIZ")
            
        #     # Execute IK if safe
        #     if user_input == 'y':
        #         group.execute(plan[1])
            
        # except rospy.ServiceException as e:
        #     print("Service call failed: %s"%e)
        

        current_angles = limb.joint_angles()
        print("Current joint angles: ", current_angles)

        desired_joint_pos = {
            "right_j0": current_angles["right_j0"],
            "right_j1": current_angles["right_j1"],
            "right_j2": current_angles["right_j2"],
            "right_j3": current_angles["right_j3"],
            "right_j4": current_angles["right_j4"] - 1.2,
            "right_j5": current_angles["right_j5"],
            "right_j6": current_angles["right_j6"],
        }

        print(desired_joint_pos)

        input('Press [ Enter ] for tipping: ')

        limb.move_to_joint_positions(desired_joint_pos)

        rospy.sleep(30.0)

        
        # # Open the right gripper
        # print('Opening...')
        # right_gripper.open()
        # rospy.sleep(1.0)
        # print('Done!')

                #         # Python's syntax for a main() method# Construct the request
        # request = GetPositionIKRequest()
        # request.ik_request.group_name = "right_arm"

        # input('Press [ Enter ] for fourth movement: ')

        # # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
        # # link = "stp_022312TP99620_tip_1"
        # link = "right_gripper_tip"

        # request.ik_request.ik_link_name = link
        # # request.ik_request.attempts = 20
        # request.ik_request.pose_stamped.header.frame_id = "base"
        
        # Set the desired orientation for the end effector HERE
        # request.ik_request.pose_stamped.pose.position.x = type_loc[0]
        # request.ik_request.pose_stamped.pose.position.y = type_loc[1] 
        # request.ik_request.pose_stamped.pose.position.z = type_loc[2] 

        # q_initial = [0, 1, 0 ,0]
        # theta = -90 * (3.141592653/180.0)
        # q_rot = [np.cos(theta/2), np.sin(theta/2),0,0]
        # q_new = tf.quaternion_multiply(q_rot, q_initial)


        # request.ik_request.pose_stamped.pose.position.x = 0.687 
        # request.ik_request.pose_stamped.pose.position.y = 0.159 - 0.47
        # request.ik_request.pose_stamped.pose.position.z = 0.383 + 0.47   
        # request.ik_request.pose_stamped.pose.orientation.x = 0
        # request.ik_request.pose_stamped.pose.orientation.y = 1
        # request.ik_request.pose_stamped.pose.orientation.z = 0
        # request.ik_request.pose_stamped.pose.orientation.w = 0


        # request.ik_request.pose_stamped.pose.position.x = 1.0126691798806136
        # request.ik_request.pose_stamped.pose.position.y = 0.1599
        # request.ik_request.pose_stamped.pose.position.z = 0.287   
        # request.ik_request.pose_stamped.pose.orientation.x = 0.597
        # request.ik_request.pose_stamped.pose.orientation.y = -0.497
        # request.ik_request.pose_stamped.pose.orientation.z = 0.483
        # request.ik_request.pose_stamped.pose.orientation.w = -0.4033
            
        # try:
        #     # Send the request to the service
        #     response = compute_ik(request)
            
        #     # Print the response HERE
        #     print(response)
        #     group = MoveGroupCommander("right_arm")

        #     # Setting position and orientation target
        #     group.set_pose_target(request.ik_request.pose_stamped)

        #     # TRY THIS
        #     # Setting just the position without specifying the orientation
        #     # group.set_position_target([ik_class.goal_x, ik_class.goal_y, ik_class.goal_z])

        #     # Plan IK
        #     plan = group.plan()
        #     user_input = input("Enter 'y' if the trajectory looks safe on RVIZ")
            
        #     # Execute IK if safe
        #     if user_input == 'y':
        #         group.execute(plan[1])
            
        # except rospy.ServiceException as e:
        #     print("Service call failed: %s"%e)

# Python's syntax for a main() method
if __name__ == '__main__':
    main()

