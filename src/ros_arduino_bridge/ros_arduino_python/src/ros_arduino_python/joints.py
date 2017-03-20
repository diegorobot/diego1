#!/usr/bin/env python

# Copyright (c) 2017-2018 diego. 
# All right reserved.
#

## @file joints.py the jonit clase support functions for joints.

## @brief Joints hold current values.
from ros_arduino_msgs.srv import *
class Joint:

    ## @brief Constructs a Joint instance.
    ##
    ## @param servoNum The servo id for this joint.
    ## 
    ## @param name The joint name.
    ## 
    ## @param name The servo control range.
    def __init__(self, name, servoNum, range):
        self.name = name
        self.servoNum=servoNum
        self.range=range

        self.position = 0.0
        self.velocity = 0.0
        self.last = rospy.Time.now()

    ## @brief Set the current position.
    def setCurrentPosition(self):
        rospy.wait_for_service('/arduino/servo_write')
	    try:
	        servo_write=rospy.ServiceProxy('/arduino/servo_write',ServoWrite)
	        servo_write(self.servoNum,self.position)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e   

    ## @brief Set the current position.
    ##
    ## @param position The current position. 
    def setCurrentPosition(self, position):
        rospy.wait_for_service('/arduino/servo_write')
	    try:
	        servo_write=rospy.ServiceProxy('/arduino/servo_write',ServoWrite)
	        servo_write(self.servoNum,position)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e   
      


import rospy
import xml.dom.minidom

from math import pi, radians

## @brief Get joint parameters from URDF
def getJointsFromURDF():
    try:
        description = rospy.get_param("robot_description")
        robot = xml.dom.minidom.parseString(description).getElementsByTagName('robot')[0]
        joints = {}
        # Find all non-fixed joints
        for child in robot.childNodes:
            if child.nodeType is child.TEXT_NODE:
                continue
            if child.localName == 'joint':
                jtype = child.getAttribute('type')
                if jtype == 'fixed':
                  continue
                name = child.getAttribute('name')
                if jtype == 'continuous':
                    minval = -pi
                    maxval = pi
                else:
                    limit = child.getElementsByTagName('limit')[0]
                    minval = float(limit.getAttribute('lower'))
                    maxval = float(limit.getAttribute('upper'))

                if minval > 0 or maxval < 0:
                    zeroval = (maxval + minval)/2
                else:
                    zeroval = 0

                joint = {'min':minval, 'max':maxval, 'zero':zeroval, 'value':zeroval }
                joints[name] = joint
        return joints
    except:
        rospy.loginfo('No URDF defined, proceeding with defaults')
        return dict()


## @brief Get limits of servo, from YAML, then URDF, then defaults if neither is defined.
def getJointLimits(name, joint_defaults, default_min=-150, default_max=150):
    min_angle = radians(default_min)
    max_angle = radians(default_max)
    
    try: 
        min_angle = joint_defaults[name]['min']
    except:
        pass
    try: 
        min_angle = radians(rospy.get_param("/arbotix/dynamixels/"+name+"/min_angle"))
    except:
        pass
    try: 
        min_angle = radians(rospy.get_param("/arbotix/joints/"+name+"/min_angle"))
    except:
        pass
    try: 
        min_angle = rospy.get_param("/arbotix/joints/"+name+"/min_position")
    except:
        pass

    try: 
        max_angle = joint_defaults[name]['max']
    except:
        pass
    try: 
        max_angle = radians(rospy.get_param("/arbotix/dynamixels/"+name+"/max_angle"))
    except:
        pass
    try: 
        max_angle = radians(rospy.get_param("/arbotix/joints/"+name+"/max_angle"))
    except:
        pass
    try: 
        max_angle = rospy.get_param("/arbotix/joints/"+name+"/max_position")
    except:
        pass

    return (min_angle, max_angle)

