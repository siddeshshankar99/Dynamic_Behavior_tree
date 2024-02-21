#! /usr/bin/env python3

import sys
import copy
import rospy
from geometry_msgs.msg import Pose, PoseStamped
import moveit_msgs.msg
import moveit_commander
import actionlib
import behavior_tree_core.msg

class BTAction(object):
    # create messages that are used to publish feedback/result
    _feedback = behavior_tree_core.msg.BTFeedback()
    _result   = behavior_tree_core.msg.BTResult()
  
    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, behavior_tree_core.msg.BTAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        moveit_commander.roscpp_initialize(sys.argv)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "right_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)
    
        self.box_name = "object1"
        self.scene = scene
        self.robot = robot

  
    def execute_cb(self, goal):
        # publish info to the console for the user
        rospy.loginfo('Checking Condition')
        loop_executed = False
        box_name = self.box_name
        scene = self.scene
        
        # Define your tolerance (epsilon) for each axis
        epsilon_x = 0.1
        epsilon_y = 0.1
        epsilon_z = 0.1
    
        while not rospy.is_shutdown() and not loop_executed:
            print(pose_stamped_callback.pose_stamped)
            # check if the button is in the goal position
            if abs(pose_stamped_callback.pose_stamped.position.x - 0.4) <= epsilon_x and abs(pose_stamped_callback.pose_stamped.position.y - (-1.5)) <= epsilon_y and abs(pose_stamped_callback.pose_stamped.position.z - 0.1) <= epsilon_z:
            
                success = True
                self.set_status('SUCCESS')
                print('success')
            else:
                self.set_status('FAILURE')
               
            rospy.sleep(1)
            loop_executed = True
        return
  
    def set_status(self, status):
        if status == 'SUCCESS':
            self._feedback.status = 1
            self._result.status = self._feedback.status
            rospy.loginfo('Action %s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
        elif status == 'FAILURE':
            self._feedback.status = 2
            self._result.status = self._feedback.status
            rospy.loginfo('Action %s: Failed' % self._action_name)
            self._as.set_succeeded(self._result)
        else:
            rospy.logerr('Action %s: has a wrong return status' % self._action_name)

def pose_stamped_callback(msg):
    # Process the PoseStamped message
    pose_stamped_callback.pose_stamped = msg.pose
    
if __name__ == '__main__':
    rospy.init_node('Solder_cond')
    #rospy.init_node("condition")
    BTAction(rospy.get_name())
    pose_stamped_callback.pose_stamped = Pose()
    rospy.Subscriber("/button6_pose", PoseStamped, pose_stamped_callback)
    rospy.spin()

