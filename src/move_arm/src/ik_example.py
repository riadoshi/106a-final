#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
import numpy as np
from numpy import linalg
from geometry_msgs.msg import Point, PointStamped
import sys

class IKExample():

    def __init__(self):

        self.goal_point_sub = rospy.Subscriber("/goal_point", Point, self.goal_img_callback) 
    

    def goal_img_callback(self, msg):
        self.goal_x = msg.x 
        self.goal_y = msg.y 
        self.goal_z = msg.z



def main():
    # Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
    rospy.init_node('service_query')
    # Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    # self.goal_point_sub = rospy.Subscriber("/goal_point", Point, None)

    ik_class = IKExample()

    while not rospy.is_shutdown():
        
        
        # Construct the request
        request = GetPositionIKRequest()
        request.ik_request.group_name = "right_arm"

        input('Press [ Enter ]: ')

        #8.5 x
        # y = 0
        # z=-3
        #9 deg

        # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
        link = "stp_022312TP99620_tip_1"
        #link = "right_gripper_tip"

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
        rospy.sleep(1.0)

# Python's syntax for a main() method
if __name__ == '__main__':
    main()
