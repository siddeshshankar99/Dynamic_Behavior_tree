#! /usr/bin/env python3

import sys
import rospy
import tf2_geometry_msgs
import moveit_msgs.msg 
from moveit_msgs.msg import Grasp
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
import trajectory_msgs.msg 
import geometry_msgs.msg 
import moveit_commander
import math
import actionlib
import behavior_tree_core.msg
import threading
from std_msgs.msg import String


class Pick1_BT(object):
  # create messages that are used to publish feedback/result
  _feedback = behavior_tree_core.msg.BTFeedback()
  _result   = behavior_tree_core.msg.BTResult()

  def __init__(self, Pick_BT):
    super(Pick1_BT, self).__init__()
    self._action_name = Pick_BT
    self._as = actionlib.SimpleActionServer(self._action_name, behavior_tree_core.msg.BTAction, execute_cb=self.execute_cb, auto_start = False)
    moveit_commander.roscpp_initialize(sys.argv)
    
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "left_arm"
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
	    # Add both finger joints of the panda robot.
        posture.joint_names = ["left_arm_finger_joint1", "left_arm_finger_joint2"]
		# Set them as open, wide enough for the object to fit.
        point = JointTrajectoryPoint()
        point.positions = [0.05, 0.05]
        point.time_from_start = rospy.Duration(0.5)
        posture.points = [point]


    def closed_gripper(posture):
       # Add both finger joints of the panda robot.
       posture.joint_names = ["left_arm_finger_joint1", "left_arm_finger_joint2"]

       # Set them as closed. 
       point = JointTrajectoryPoint()
       point.positions = [0.0, 0.0]
       point.time_from_start = rospy.Duration(0.5)
       posture.points = [point]


    def pickRobot(loop_executed):
        #Executing without any preempt request
        while not self.stop_event.is_set() and not loop_executed:
            rospy.loginfo('Executing Action')
            move_group = self.move_group
            box1_name = self.box1_name
            box2_name = self.box2_name
            box3_name = self.box3_name
            scene = self.scene
            
            
            #obstruct1
            attached_objects1 = scene.get_attached_objects([box1_name])
            is_attached1 = len(attached_objects1.keys()) > 0 
            print(is_attached1)
            is_known1 = box1_name in scene.get_known_object_names()
            print(is_known1)
            
            #obstruct2
            attached_objects2 = scene.get_attached_objects([box2_name])
            is_attached2 = len(attached_objects2.keys()) > 0 
            print(is_attached2)
            is_known2 = box2_name in scene.get_known_object_names()
            print(is_known2)
            
            #obstruct3
            attached_objects3 = scene.get_attached_objects([box3_name])
            is_attached3 = len(attached_objects3.keys()) > 0 
            print(is_attached3)
            is_known3 = box3_name in scene.get_known_object_names()
            print(is_known3)
            
            if (is_attached1 and not is_known1)  or (is_attached2 and not is_known2) or (is_attached3 and not is_known3):
                success = True
                rospy.sleep(0.1)
                loop_executed = True 
                self.set_status('SUCCESS')
                
            else:  
                grasps = Grasp()
                # Setting grasp pose
                grasps.grasp_pose.header.frame_id = "left_arm_link0"
                grasps.grasp_pose.pose.orientation.w = 0
                grasps.grasp_pose.pose.orientation.x = -0.924 + pose_stamped_callback.pose_stamped.orientation.x
                grasps.grasp_pose.pose.orientation.y = -0.383 + pose_stamped_callback.pose_stamped.orientation.y     
                grasps.grasp_pose.pose.orientation.z = 0 + pose_stamped_callback.pose_stamped.orientation.z
                grasps.grasp_pose.pose.position.x = pose_stamped_callback.pose_stamped.position.x
                grasps.grasp_pose.pose.position.y = pose_stamped_callback.pose_stamped.position.y
                grasps.grasp_pose.pose.position.z = 0.2
                # Setting pre-grasp approach
                grasps.pre_grasp_approach.direction.header.frame_id = "left_arm_link0"
                grasps.pre_grasp_approach.direction.vector.z = -1.0
                grasps.pre_grasp_approach.min_distance = 0.175
                grasps.pre_grasp_approach.desired_distance = 0.25
                # Setting post-grasp retreat
                grasps.post_grasp_retreat.direction.header.frame_id = "left_arm_link0"
                grasps.post_grasp_retreat.direction.vector.z = 1.0
                grasps.post_grasp_retreat.min_distance = 0.2
                grasps.post_grasp_retreat.desired_distance = 0.25
                # Setting posture of eef before grasp
                open_gripper(grasps.pre_grasp_posture)
                # Setting posture of eef during grasp
                closed_gripper(grasps.grasp_posture)
                # Call pick to pick up the object using the grasps given
                rospy.loginfo('Calling move_group.pick()')
                move_group.pick("obstruct1", grasps)
                rospy.loginfo('Finished move_group.pick()')
               
                if self._as.is_preempt_requested():
                    self.stop_event.set()
                    return
                    
                move_group.stop()
                scene = self.scene
                box1_name = self.box1_name
                box2_name = self.box2_name
                box3_name = self.box3_name
                
                #obstruct1
                attached_objects1 = scene.get_attached_objects([box1_name])
                is_attached1 = len(attached_objects1.keys()) > 0 
                print(is_attached1)
                is_known1 = box1_name in scene.get_known_object_names()
                print(is_known1)
            
                #obstruct2
                attached_objects2 = scene.get_attached_objects([box2_name])
                is_attached2 = len(attached_objects2.keys()) > 0 
                print(is_attached2)
                is_known2 = box2_name in scene.get_known_object_names()
                print(is_known2)
            
                #obstruct3
                attached_objects3 = scene.get_attached_objects([box3_name])
                is_attached3 = len(attached_objects3.keys()) > 0 
                print(is_attached3)
                is_known3 = box3_name in scene.get_known_object_names()
                print(is_known3)
                
                
                if (is_attached1 and not is_known1)  or (is_attached2 and not is_known2) or (is_attached3 and not is_known3):
                    success = True
                    self.set_status('SUCCESS')
                else:
                    self.set_status('FAILURE')
                if self._as.is_preempt_requested():
                    self.stop_event.set()
                    return
                rospy.sleep(0.1)
                loop_executed = True 
             
        return loop_executed
    
    robot_thread = threading.Thread(target=pickRobot, args=(loop_executed,))
    robot_thread.start()
    
	
    while self._as.is_active():

        if self._as.is_preempt_requested():
          rospy.loginfo('Action Halted')
          move_group = self.move_group
          move_group.stop()
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
        self._as.set_aborted(self._result)
      else:
        rospy.logerr('Action %s: has a wrong return status' % self._action_name)
        	
def pose_stamped_callback(msg):
    # Process the PoseStamped message
    pose_stamped_callback.pose_stamped = msg.pose
    

def main():
    rospy.init_node("PickO_L")
    #rospy.init_node("action")
    Pick1_BT(rospy.get_name())
    pose_stamped_callback.pose_stamped = Pose()
    rospy.Subscriber("/obstruct1_pose", PoseStamped, pose_stamped_callback)
    rospy.Subscriber("/obstruct2_pose", PoseStamped, pose_stamped_callback)
    rospy.Subscriber("/obstruct3_pose", PoseStamped, pose_stamped_callback)
    rospy.spin()


if __name__ == "__main__":
    main()

