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
        self.rate = 20.0
        
        # left Arm jonits list
        self.left_shoulder_stevo_to_axis=Joint(left_shoulder_stevo_to_axis,6,PI)
        self.left_shoulder_stevo_lift_to_axis=Joint(left_shoulder_stevo_lift_to_axis,7,PI)
        self.left_big_arm_up_to_axis=Joint(left_big_arm_up_to_axis,8,PI)
        self.left_small_arm_up_to_axis=Joint(left_small_arm_up_to_axis,9,PI)
        self.left_wrist_run_stevo_to_axis=Joint(left_wrist_run_stevo_to_axis,10,PI)
                
        
        # left hand joint
        self.left_hand_run_stevo_to_left_hand_run_stevo_axis=Joint(left_hand_run_stevo_to_left_hand_run_stevo_axis,11,PI)
        
        # right Arm jonits
        self.right_shoulder_stevo_to_axis=Joint(right_shoulder_stevo_to_axis,0,PI)
        self.right_shoulder_stevo_lift_to_axis=Joint(right_shoulder_stevo_lift_to_axis,1,PI)
        self.right_big_arm_up_to_axis=Joint(right_big_arm_up_to_axis,2,PI)
        self.right_small_arm_up_to_axis=Joint(right_small_arm_up_to_axis,3,PI)
        self.right_wrist_run_stevo_to_axis=Joint(right_wrist_run_stevo_to_axis,4,PI)
        
        # left hand joint
        self.right_hand_run_stevo_to_right_hand_run_stevo_axis=Joint(right_hand_run_stevo_to_right_hand_run_stevo_axis,5,PI)
        
        # set the left arm back to the resting position
        rospy.loginfo("set the left arm back to the resting position")
        self.left_shoulder_stevo_to_axis.setCurrentPosition(PI/2)
        self.left_shoulder_stevo_lift_to_axis.setCurrentPosition(PI/2)
        self.left_big_arm_up_to_axis.setCurrentPosition(PI/2)
        self.left_small_arm_up_to_axis.setCurrentPosition(PI/2)
        self.left_wrist_run_stevo_to_axis.setCurrentPosition(PI/2)
        # set the right arm back to the resting position
        rospy.loginfo("set the right arm back to the resting position")
        self.right_shoulder_stevo_to_axis.setCurrentPosition(PI/2)
        self.right_shoulder_stevo_lift_to_axis.setCurrentPosition(PI/2)
        self.right_big_arm_up_to_axis.setCurrentPosition(PI/2)
        self.right_small_arm_up_to_axis.setCurrentPosition(PI/2)
        self.right_wrist_run_stevo_to_axis.setCurrentPosition(PI/2)
        # set the left hand back to the resting position
        rospy.loginfo("set the left hand back to the resting position")
        self.left_hand_run_stevo_to_left_hand_run_stevo_axis.setCurrentPosition(PI/2)
        # set the right hand back to the resting position
        rospy.loginfo("set the right hand back to the resting position")
        self.right_hand_run_stevo_to_right_hand_run_stevo_axis.setCurrentPosition(PI/2)

        # action server
        self.server = actionlib.SimpleActionServer('follow_joint_trajectory', FollowJointTrajectoryAction, execute_cb=self.actionCb, auto_start=True)
        rospy.loginfo("Started FollowController")


    def actionCb(self, goal):
        rospy.loginfo(self.name + ": Action goal recieved.")
        traj = goal.trajectory

        if set(self.joints) != set(traj.joint_names):
            for j in self.joints:
                if j not in traj.joint_names:
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
            indexes = [traj.joint_names.index(joint) for joint in self.joints]
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
            indexes = [traj.joint_names.index(joint) for joint in self.joints]
        except ValueError as val:
            rospy.logerr("Invalid joint in trajectory.")
            return False

        # get starting timestamp, MoveIt uses 0, need to fill in
        start = traj.header.stamp
        if start.secs == 0 and start.nsecs == 0:
            start = rospy.Time.now()

        r = rospy.Rate(self.rate)
        last = [ self.device.joints[joint].position for joint in self.joints ]
        for point in traj.points:
            while rospy.Time.now() + rospy.Duration(0.01) < start:
                rospy.sleep(0.01)
            desired = [ point.positions[k] for k in indexes ]
            endtime = start + point.time_from_start
            while rospy.Time.now() + rospy.Duration(0.01) < endtime:
                err = [ (d-c) for d,c in zip(desired,last) ]
                velocity = [ abs(x / (self.rate * (endtime - rospy.Time.now()).to_sec())) for x in err ]
                rospy.logdebug(err)
                for i in range(len(self.joints)):
                    if err[i] > 0.001 or err[i] < -0.001:
                        cmd = err[i] 
                        top = velocity[i]
                        if cmd > top:
                            cmd = top
                        elif cmd < -top:
                            cmd = -top
                        last[i] += cmd
                        self.device.joints[self.joints[i]].setControlOutput(last[i])
                    else:
                        velocity[i] = 0
                r.sleep()
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

