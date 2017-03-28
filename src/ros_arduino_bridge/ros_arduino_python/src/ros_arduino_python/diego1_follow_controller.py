#!/usr/bin/env python

# Copyright (c) 2017-2018 williamar. 
# All right reserved.

import rospy, actionlib

from control_msgs.msg import FollowJointTrajectoryAction
from trajectory_msgs.msg import JointTrajectory
from diagnostic_msgs.msg import *
from math import pi as PI, degrees, radians

class FollowController:

    def __init__(self, name):
        self.name = name

        # rates
        self.rate = 10.0
        
        # Arm jonits
        self.arm_base_to_arm_round_joint_stevo0=Joint(arm_base_to_arm_round_joint_stevo0,0,1.5797,-1.5707,130,0,False)
        self.shoulder_2_to_arm_joint_stevo1=Joint(shoulder_2_to_arm_joint_stevo1,1,1.5707,-0.1899,115,45,False)
        self.big_arm_round_to_joint_stevo2=Joint(big_arm_round_to_joint_stevo2,2,2.5891,1,130,45,False)
        self.arm_joint_stevo2_to_arm_joint_stevo3=Joint(arm_joint_stevo2_to_arm_joint_stevo3,3,1.5707,-1.5707,130,0,False)
        self.wrist_to_arm_joint_stevo4=Joint(wrist_to_arm_joint_stevo4,4,1.5707,-1.5707,130,0,False)
        self.arm_joint_stevo4_to_arm_joint_stevo5=Joint(arm_joint_stevo4_to_arm_joint_stevo5,5,1.5707,-1.5707,130,0,True)
        
        # gripper joint
        self.hand_to_grip_joint_stevo6=Joint(hand_to_grip_joint_stevo6,6,0.8285,-0.1762,140,0,False)
        self.hand_to_grip_joint_stevo7=Joint(hand_to_grip_joint_stevo7,7,0.1584,-0.8285,140,0,False)
        
        self.joints=[arm_base_to_arm_round_joint_stevo0,
        shoulder_2_to_arm_joint_stevo1,
        big_arm_round_to_joint_stevo2,
        arm_joint_stevo2_to_arm_joint_stevo3,
        wrist_to_arm_joint_stevo4,
        arm_joint_stevo4_to_arm_joint_stevo5,
        hand_to_grip_joint_stevo6,
        hand_to_grip_joint_stevo7]
        
        # set the left arm back to the resting position
        rospy.loginfo("set the arm back to the default pose")

        # set the left hand back to the resting position
        rospy.loginfo("set the gripper back to the default pose")

        # action server
        self.server = actionlib.SimpleActionServer('follow_joint_trajectory', FollowJointTrajectoryAction, execute_cb=self.actionCb, auto_start=True)
        rospy.loginfo("Started FollowController")


    def actionCb(self, goal):
        rospy.loginfo(self.name + ": Action goal recieved.")
        traj = goal.trajectory

        if set(self.joints) != set(traj.joint_names):
            for j in self.joints:
                if j.name not in traj.joint_names:
                    msg = "Trajectory joint names does not match action controlled joints." + str(traj.joint_names)
                    rospy.logerr(msg)
                    self.server.set_aborted(text=msg)
                    return
            rospy.logwarn("Extra joints in trajectory")

        if not traj.points:
            msg = "Trajectory empy."
            rospy.logerr(msg)
            self.server.set_aborted(text=msg)
            return

        try:
            indexes = [traj.joint_names.index(joint.name) for joint in self.joints]
        except ValueError as val:
            msg = "Trajectory invalid."
            rospy.logerr(msg)
            self.server.set_aborted(text=msg)
            return

        if self.executeTrajectory(traj):   
            self.server.set_succeeded()
        else:
            self.server.set_aborted(text="Execution failed.")

        rospy.loginfo(self.name + ": Done.")
    
    def commandCb(self, msg):
        # don't execute if executing an action
        if self.server.is_active():
            rospy.loginfo(self.name+": Received trajectory, but action is active")
            return
        self.executing = True
        self.executeTrajectory(msg)
        self.executing = False    

    def executeTrajectory(self, traj):
        rospy.loginfo("Executing trajectory")
        rospy.logdebug(traj)
        # carry out trajectory
        try:
            indexes = [traj.joint_names.index(joint.name) for joint in self.joints]
        except ValueError as val:
            rospy.logerr("Invalid joint in trajectory.")
            return False

        # get starting timestamp, MoveIt uses 0, need to fill in
        start = traj.header.stamp
        if start.secs == 0 and start.nsecs == 0:
            start = rospy.Time.now()

        r = rospy.Rate(self.rate)
	for point in traj.points:            
            desired = [ point.positions[k] for k in indexes ]#期望的控制点
            for i in indexes
                #self.joints[i].position=desired[i]#控制点对应的舵机的位置
                self.joints[i].setCurrentPosition(self.joints[i].position)#发送舵机的控制命令

            while rospy.Time.now() + rospy.Duration(0.01) < start:#如果当前时间小于舵机这个点预期完成时间，则等待
                rospy.sleep(0.01)
        return True

    def active(self):
        """ Is controller overriding servo internal control? """
        return self.server.is_active() or self.executing

    def getDiagnostics(self):
        """ Get a diagnostics status. """
        msg = DiagnosticStatus()
        msg.name = self.name
        msg.level = DiagnosticStatus.OK
        msg.message = "OK"
        if self.active():
            msg.values.append(KeyValue("State", "Active"))
        else:
            msg.values.append(KeyValue("State", "Not Active"))
        return msg
        
    def initRightArm():
	    r = rospy.Rate(10)
	    servoWrite(0,radians(100)) 
	    r.sleep()
	    servoWrite(1,radians(80))
	    r.sleep()
	    servoWrite(2,radians(90))
	    r.sleep()
	    servoWrite(3,radians(50))
	    r.sleep()
	    servoWrite(4,radians(90))
	    r.sleep()
	    servoWrite(5,radians(90))
	    r.sleep()

    def initLeftArm():	
	    r = rospy.Rate(10)
	    servoWrite(6,radians(150))
	    #r.sleep()
	    #servoWrite(7,radians(90))
	    #r.sleep()
	    #servoWrite(8,radians(90))
	    #r.sleep()
	    #servoWrite(9,radians(90))
	    #r.sleep()
	    #servoWrite(10,radians(90))
	    #r.sleep()
	    #servoWrite(11,radians(90))
	    #r.sleep()     

