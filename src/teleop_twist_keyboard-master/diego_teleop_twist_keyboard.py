#!/usr/bin/env python
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist
from ros_arduino_msgs.srv import *
from math import pi as PI, degrees, radians
import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Twist, Servo or sensor!
---------------------------
Moving around:
   u    i    o
   j    k    l

, : up (+z)
. : down (-z)

----------------------------
Left arm servo control:
+   1   2   3   4   5   6
-   q   w   e   r   t   y  
----------------------------
Right arm servo control:
+   a   s   d   f   g   h
-   z   x   c   v   b   n 

p : init the servo

CTRL-C to quit
"""

moveBindings = {
		'i':(1,0,0,0),
		'o':(1,0,0,1),
		'j':(-1,0,0,-1),
		'l':(-1,0,0,1),
		'u':(1,0,0,-1),
		'k':(-1,0,0,0),
		',':(0,0,1,0),
		'.':(0,0,-1,0),
	       }

armServos={
      	        '1':(0,1),
	        '2':(1,1),
	        '3':(2,1),
	        '4':(3,1),
	        '5':(4,1),
	        '6':(5,1),
	        'a':(6,1),
	        'q':(0,0),
	        'w':(1,0),
	        'e':(2,0),
	        'r':(3,0),
	        't':(4,0),
	        'y':(5,0),
	        'z':(6,0),
	      }
	      
armServoValues=[70,100,60,80,70,70,35]

def getradians(angle):
	return PI*angle/180


def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

       
def servoWrite(servoNum, value):
        rospy.wait_for_service('/arduino/servo_write')
	try:
	    servo_write=rospy.ServiceProxy('/arduino/servo_write',ServoWrite)
	    servo_write(servoNum,value)
	    print servoNum
            print value
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
    
	 
def vels(speed,turn):
	return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    	settings = termios.tcgetattr(sys.stdin)
	
	pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
	rospy.init_node('teleop_twist_keyboard')

	speed = rospy.get_param("~speed", 0.25)
	turn = rospy.get_param("~turn", 1.0)
	x = 0
	y = 0
	z = 0
	th = 0
	status = 0	

	try:
		print msg
		print vels(speed,turn)
		servoWrite(0, radians(armServoValues[0]))
		servoWrite(1, radians(armServoValues[1]))
		servoWrite(2, radians(armServoValues[2]))
		servoWrite(3, radians(armServoValues[3]))
		servoWrite(4, radians(armServoValues[4]))
		servoWrite(5, radians(armServoValues[5]))
		servoWrite(6, radians(armServoValues[6]))
		while(1):
			key = getKey()
			print key
			if key in moveBindings.keys():
				x = moveBindings[key][0]
				y = moveBindings[key][1]
				z = moveBindings[key][2]
				th = moveBindings[key][3]
				twist = Twist()
				twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed;
				twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
				pub.publish(twist)
			elif key in armServos.keys():  
			    if(armServos[key][1]==0):
			        armServoValues[armServos[key][0]]=armServoValues[armServos[key][0]]-5
			        if armServoValues[armServos[key][0]]<=0:
			           armServoValues[armServos[key][0]]=0
			    else:
			        armServoValues[armServos[key][0]]=armServoValues[armServos[key][0]]+5
			        if armServoValues[armServos[key][0]]>=180:
			           armServoValues[armServos[key][0]]=180
			    print armServoValues[armServos[key][0]]
			    servoWrite(armServos[key][0], radians(armServoValues[armServos[key][0]]))
			else:
				x = 0
				y = 0
				z = 0
				th = 0
				if (key == '\x03'):
					break

			

	except BaseException,e:
		print e

	finally:
		twist = Twist()
		twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
		twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
		pub.publish(twist)

    		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


