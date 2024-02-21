#! /usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

import actionlib

import behavior_tree_core.msg




class BTAction(object):
  # create messages that are used to publish feedback/result
  _feedback = behavior_tree_core.msg.BTFeedback()
  _result   = behavior_tree_core.msg.BTResult()

  def __init__(self, name):
    self._action_name = name
    self._as = actionlib.SimpleActionServer(self._action_name, behavior_tree_core.msg.BTAction, execute_cb=self.execute_cb, auto_start = False)
    self._as.start()
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "right_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    
    self.box_name = "object2"
    self.scene = scene
    self.robot = robot
    
  def execute_cb(self, goal):
    
    # publish info to the console for the user
    rospy.loginfo('Checking Condition')
    
    box_name = self.box_name
    scene = self.scene
    loop_executed = False
    
    while not rospy.is_shutdown() and not loop_executed:
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0 
            print(is_attached)
            
            is_known = box_name in scene.get_known_object_names()
            print(is_known)


            # Test if we are in the expected state
            if is_attached == False and (is_known == True):
                success = True
                self.set_status('SUCCESS')
            else: 
                self.set_status('FAILURE')

            rospy.sleep(0.1)
            loop_executed = True
            
    return 
    
    

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



if __name__ == '__main__':
  rospy.init_node('button2_not_condition_R')
  #rospy.init_node("condition")
  BTAction(rospy.get_name())
  rospy.spin()
