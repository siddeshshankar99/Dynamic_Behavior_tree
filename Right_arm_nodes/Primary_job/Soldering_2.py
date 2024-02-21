#! /usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import actionlib
import behavior_tree_core.msg
import math
import threading

class move1_BT(object):
  # create messages that are used to publish feedback/result
  _feedback = behavior_tree_core.msg.BTFeedback()
  _result   = behavior_tree_core.msg.BTResult()

  def __init__(self, move_BT):
    super(move1_BT, self).__init__()
    self._action_name = move_BT
    self._as = actionlib.SimpleActionServer(self._action_name, behavior_tree_core.msg.BTAction, execute_cb=self.execute_cb, auto_start = False)
    moveit_commander.roscpp_initialize(sys.argv)
    
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    object_pose_pub = rospy.Publisher(
        "/button_pose", geometry_msgs.msg.Pose, queue_size=20)
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
    self.object_pose_pub = object_pose_pub
    self.eef_link = eef_link
    
  def execute_cb(self, goal):
    
    r = rospy.Rate(1)
    loop_executed = False
    
    # publish info to the console for the user
    rospy.loginfo('Starting Action')
    self.stop_event = threading.Event()
    
    # Check the ROS parameter to see if the robot has already reached its destination
    if rospy.get_param('~has_reached_destination', False):
       rospy.loginfo('Robot has already reached its destination. Not executing again.')
       success = True
       self.set_status('SUCCESS')
       return 
    

    def runRobot(loop_executed):
      #Executing without any preempt request
      while not self.stop_event.is_set() and not loop_executed:
          rospy.loginfo('Executing Action')
          move_group = self.move_group
          pose_goal = geometry_msgs.msg.Pose()
          pose_goal.orientation.w = 0
          pose_goal.orientation.x = -0.924
          pose_goal.orientation.y = -0.383
          pose_goal.orientation.z = 0
          pose_goal.position.x = 0.45
          pose_goal.position.y = -1.6
          pose_goal.position.z = 0.2
            
          pose = self.object_pose_pub.publish(pose_goal)

          move_group.set_pose_target(pose_goal)
          goal_success = move_group.go(wait=True)

          if self._as.is_preempt_requested():
            self.stop_event.set()
            return
            
          move_group.stop()
          move_group.clear_pose_targets()
          current_pose = self.move_group.get_current_pose().pose
          current_pose_rounded = geometry_msgs.msg.Pose()       
          current_pose_rounded.position.x = round(current_pose.position.x, 2)
          current_pose_rounded.position.y = round(current_pose.position.y, 2)
          current_pose_rounded.position.z = round(current_pose.position.z, 2)
          current_pose_rounded.orientation.w = round(current_pose.orientation.w, 1)
          current_pose_rounded.orientation.x = -0.924
          current_pose_rounded.orientation.y = -0.383
          current_pose_rounded.orientation.z = 0
          print(current_pose_rounded)
          
          if pose_goal == current_pose_rounded:
            success = True
            self.set_status('SUCCESS')
            # Set the ROS parameter once the robot reaches its destination
            rospy.set_param('~has_reached_destination', True)
          else:
            self.set_status('FAILURE') 			      
          
          if self._as.is_preempt_requested():
            self.stop_event.set()
            return
            
          loop_executed = True
          
      return

    robot_thread = threading.Thread(target=runRobot, args=(loop_executed,))
    robot_thread.start()


        # check that preempt has not been requested by the client
    while self._as.is_active():

        if self._as.is_preempt_requested():
          rospy.loginfo('Action Halted')
          move_group = self.move_group
          move_group.stop()
          move_group.clear_pose_targets()
          self.stop_event.set()
          robot_thread.join()
          self._as.set_preempted()
          success = False
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
        self._as.set_succeeded(self._result)
      else:
        rospy.logerr('Action %s: has a wrong return status' % self._action_name)
  


def main():
  #rospy.init_node('action')
  rospy.init_node('Solder_2')
  move1_BT(rospy.get_name())
  rospy.spin()
  
  
if __name__ == '__main__':
  main()
  
