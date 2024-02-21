#!/usr/bin/env python3

import sys
import rospy
import tf2_geometry_msgs
import moveit_msgs.msg
from moveit_msgs.msg import PlaceLocation
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, Quaternion
import trajectory_msgs.msg 
import geometry_msgs.msg 
import moveit_commander
import math
import actionlib
import behavior_tree_core.msg
import threading
from std_msgs.msg import String
import random

class Place1_BT(object):
  # create messages that are used to publish feedback/result
  _feedback = behavior_tree_core.msg.BTFeedback()
  _result   = behavior_tree_core.msg.BTResult()

  def __init__(self, Place_BT):
    super(Place1_BT, self).__init__()
    self._action_name = Place_BT
    self._as = actionlib.SimpleActionServer(self._action_name, behavior_tree_core.msg.BTAction, execute_cb=self.execute_cb, auto_start = False)
    moveit_commander.roscpp_initialize(sys.argv)
    
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "right_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )
    planning_frame = move_group.get_planning_frame()
    eef_link = move_group.get_end_effector_link()
    group_names = robot.get_group_names()
    
    self._as.start()
    self.move_group = move_group
    
    self.eef_link = eef_link
    self.scene = scene
    self.box1_name = "obstruct1"
    self.box2_name = "obstruct2"
    self.box3_name = "obstruct3"
    


  def execute_cb(self, goal):
    # helper variables
    r = rospy.Rate(1)
    loop_executed = False
    
    # publish info to the console for the user
    rospy.loginfo('Starting Action')
    self.stop_event = threading.Event()

    def open_gripper(posture):
        # Add both finger joints of panda robot.
        posture.joint_names = ["right_arm_finger_joint1", "right_arm_finger_joint2"]

        # Set them as open, wide enough for the object to fit.
        point = JointTrajectoryPoint()
        point.positions = [0.05, 0.05]
        point.time_from_start = rospy.Duration(0.5)

        posture.points = [point]


    def closed_gripper(posture):
        # Add both finger joints of panda robot.
        posture.joint_names = ["right_arm_finger_joint1", "right_arm_finger_joint2"]

        # Set them as closed.
        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.0]
        point.time_from_start = rospy.Duration(0.5)

        posture.points = [point]


    def placeRobot(loop_executed):
        #Executing without any preempt request
        while not self.stop_event.is_set() and not loop_executed:
            rospy.loginfo('Executing Action')
            move_group = self.move_group
            scene = self.scene
            box1_name = self.box1_name
            box2_name = self.box2_name
            box3_name = self.box3_name
    
            
            attached_objects1 = scene.get_attached_objects([box1_name])
            is_attached1 = len(attached_objects1.keys()) > 0 
            print(is_attached1)
            
            is_known1 = box1_name in scene.get_known_object_names()
            print(is_known1)
            
            if is_attached1 == True and (is_known1 == False):
               # Create a list of place locations to be attempted, currently only creating a single place location.
                place_location = PlaceLocation()

                # Setting place location pose
                place_location.place_pose.header.frame_id = "left_arm_link0"
                orientation = Quaternion()
                orientation.x = 0
                orientation.y = 0
                orientation.z = 0
                orientation.w = 1
                place_location.place_pose.pose.orientation = orientation
                place_location.place_pose.pose.position.x = 0.6
                place_location.place_pose.pose.position.y = -1.2
                place_location.place_pose.pose.position.z = 0.075

                # Setting pre-place approach
                place_location.pre_place_approach.direction.header.frame_id = "left_arm_link0"
                place_location.pre_place_approach.direction.vector.z = -1.0
                place_location.pre_place_approach.min_distance = 0.125
                place_location.pre_place_approach.desired_distance = 0.275

                # Setting post-grasp retreat
                place_location.post_place_retreat.direction.header.frame_id = "left_arm_link0"
                place_location.post_place_retreat.direction.vector.z = 1.0
                place_location.post_place_retreat.min_distance = 0.15
                place_location.post_place_retreat.desired_distance = 0.25

                # Setting posture of eef after placing object
                open_gripper(place_location.post_place_posture)

                # Set support surface as table1.
                move_group.set_support_surface_name("left_arm_link0")

                # Call place to place the object using the place locations given.
                rospy.loginfo('Calling move_group.place()')
                move_group.place("obstruct1", place_location)
                rospy.loginfo('Finished move_group.place()')
                
                if self._as.is_preempt_requested():
                    self.stop_event.set()
                    return
                
                move_group.stop()
                scene = self.scene
                box1_name = self.box1_name
                
                attached_objects1 = scene.get_attached_objects([box1_name])
                is_attached1 = len(attached_objects1.keys()) > 0 
                print(is_attached1)
                
                is_known1 = box1_name in scene.get_known_object_names()
                print(is_known1)
                
                if is_attached1 == False and (is_known1 == True):
                    self.set_status('SUCCESS')
                else:
                    self.set_status('FAILURE')
                if self._as.is_preempt_requested():
                    self.stop_event.set()
                    return
                rospy.sleep(0.1)
                loop_executed = True 

        return
    
    robot_thread = threading.Thread(target=placeRobot, args=(loop_executed,))
    robot_thread.start()

    	

        # check that preempt has not been requested by the client
    while self._as.is_active():

        if self._as.is_preempt_requested():
          rospy.loginfo('Action Halted')
          move_group = self.move_group
          move_group.stop()
          self.stop_event.set()
          robot_thread.join()
          self._as.set_preempted()
          self.set_status('FAILURE')
          return 
        
    robot_thread.join()

  def set_status(self,status):
      if status == 'SUCCESS':
        self._feedback.status = 1
        self._result.status = self._feedback.status
        rospy.loginfo('Action %s: Succeeded' % self._action_name)
        self._as.set_succeeded(self._result)
      elif status == 'FAILURE':
        self._feedback.status = 2
        self._result.status = self._feedback.status
        rospy.loginfo('Action %s: Failed' % self._action_name)
        self._as.set_aborted(self._result)
      else:
        rospy.logerr('Action %s: has a wrong return status' % self._action_name)
        	

def main():
    rospy.init_node("PlaceO_R")
    #rospy.init_node("action")
    Place1_BT(rospy.get_name())
    rospy.spin()


if __name__ == "__main__":
    main()


