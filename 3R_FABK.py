#Make sure "sudo pigpiod" is invoked in terminal before start.
#	The pigpio daemon background process will not let go of GPIO pins unless the raspberry pi is restarted/rebooted.
#Servo control module.

from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory
from time import sleep

import numpy as np
import math as m


class Servo_control:
	def __init__(self, pinNum = 0): #Params. pinNum = The pin where the signal of the servo is connected. Default is 0. If zero when object is created, an exception/ error will be raised.
		PWMcorrect = 0.50 #value used to correct the servo PWM signal to achieve maxmimum rotation.
		maxPW = (2+PWMcorrect)/1000 #Set the maximum and minimum PWM signal (min = 0 degrees, max = 180 degrees)
		minPW = (1-PWMcorrect)/1000

		if not pinNum: #If the pin number is not initialized.
			raise Exception("Pin number for servo not declared") #Raise an exception to notify.
		factory = PiGPIOFactory() #Using the piGPIO factory prevents servo jitter.
		self.servo = Servo(pinNum, min_pulse_width = minPW, max_pulse_width = maxPW, pin_factory = factory)

		self.currentServoPos = 0 #Set servo initial position to be 0/90 degrees
		self.servo.value = self.currentServoPos

	def set_position(self, deg): #Param. deg = Degree position to set the servo (0 - 180 degrees)
		#Turn degree position into a value in the range between -1 and 1. 0 = -1 , 1 = 180 , 0 = 90 , etc.
		pos = ((deg/180) - 0.5) * 2 
		#Slowly write the servo to the value determined
		if pos > self.currentServoPos:
			writePos = self.currentServoPos #Get the servo's current position
			while self.currentServoPos <= pos - 0.01: #If the servo's current position is less than the position the servo need to go to. (-0.01) prevents the servo from being written to a value greater than 1 
				writePos = writePos + 0.01 #Increment the writePos of the servo by 0.01
				self.servo.value = writePos #write servo position
				self.currentServoPos = writePos #Update the servo's position
				sleep(0.002) #sleep 2ms

		elif pos < self.currentServoPos:
			writePos = self.currentServoPos
			while self.currentServoPos >= pos + 0.01:
				writePos = writePos - 0.01 #Decrement the writePos
				self.servo.value = writePos
				self.currentServoPos = writePos
				sleep(0.002)

		elif pos == self.currentServoPos:
			pass #Do nothing because we are already at that position

class Kinematics: #Kinematics class
	#All distances are in millimeters (mm)
	def __init__(self):
		#Initialize link lengths
		self.a1 = 86.80 
		self.a2 = 86.80
		self.a3 = 65.05
		
		#Servo/ joint angles
		self.J0 = 0
		self.J1 = 0
		self.J2 = 0

	#Calculates and returns the position of the specified joint given the joint angles
	def get_joint_pos(self, J_num, T0 = 0, T1 = 0, T2 = 0): #Params: 1. The joint number desired (innermost joint/servo = 0, middle = 1, outermost = 1, end-effector = 3)
															#        2, 3, 4. The theta angles that the servos are written to. T0 = servo 1 - innermost servo, ..., T2 = servo 2 - outermost servo

		#Change servo position value to degree value E.x. 90 degree servo pos = 0 degree
		#						  0 servo pos = 90 degree
		#                                                 180 servo pos = -90 degree
		#This is done because the robot's frames are drawn in a way that full-extend straight is 0 degree
		#	change relative to the base frame.
		T0 = (T0 - 90) * -1
		T1 = (T1 - 90) * -1
		T2 = (T2 - 90) * -1

		#Change angle values from degrees to radians
		rT0 = (T0 * np.pi) / 180
		rT1 = (T1 * np.pi) / 180
		rT2 = (T2 * np.pi) / 180

		#Define all "local" HTMs
		H0_1 = np.array([[m.cos(rT0), -1*m.sin(rT0), 0, self.a1*m.cos(rT0)],
				 [m.sin(rT0), m.cos(rT0), 0, self.a1*m.sin(rT0)],
				 [0, 0, 1, 0],
				 [0, 0, 0, 1]])
		H1_2 = np.array([[m.cos(rT1), -1*m.sin(rT1), 0, self.a2*m.cos(rT1)],
                                 [m.sin(rT1), m.cos(rT1), 0, self.a2*m.sin(rT1)],
                                 [0, 0, 1, 0],
                                 [0, 0, 0, 1]])
		H2_3 = np.array([[m.cos(rT2), -1*m.sin(rT2), 0, self.a3*m.cos(rT2)],
                                 [m.sin(rT2), m.cos(rT2), 0, self.a3*m.sin(rT2)],
                                 [0, 0, 1, 0],
                                 [0, 0, 0, 1]])
		
		#Find the dot product of the "local" HTMs to get the "global" HTMs
		H0_2 = H0_1.dot(H1_2)
		H0_3 = H0_2.dot(H2_3)

		#Use the joint number argument to determine what HTM to read the positions from
		x_pos, y_pos = 0
		if J_num == 0: #Joint 0 position was requested
			pass #Since joint 0 is the "origin" of the robot at pos (x = 0, y = 0)
		elif J_num == 1: #Joint 1 pos. is requested
			x_pos = H0_1[0, 3] #Get x from the displacement vector's x component of HTM 0_1
			y_pos = H0_1[1, 3] #Get y
		elif J_num == 2: #Joint 2 pos. is requested
			x_pos = H0_2[0, 3]
			y_pos = H0_2[1, 3]
		elif J_num == 3: #End-effector pos. is requested
			x_pos = H0_3[0, 3]
			y_pos = H0_3[1, 3]

		return x_pos, y_pos

	def play_forward(self): #Params are angles in degrees to set the servo of each module
		#Get the angles to be written.
		self.J0 = int(input("Joint 0 angle: "))
		self.J1 = int(input("Joint 1 angle: "))
		self.J2 = int(input("Joint 2 angle: "))

		#Set the servos to the specified angles
		servo0.set_position(self.J0)
		servo1.set_position(self.J1)
		servo2.set_position(self.J2) #Write servo position. Starting from the innermost servo to reduce backlash

		#Get the position of the end-effector.
		x, y = self.get_joint_pos(4, T0 = self.J0, T1 = self.J1, T2 = self.J2)
		print('x: ' + str(x+31.6)) #Account for distance between actual start of robot arm and the arm holder
		print('y: ' + str(y))
		print()

	def play_inverse(self, x, y): #Params are the x and y coordinate to place the end effector.
		#Declare and/or Initialize essential variables
		J0_ang = 0 #Joint 1 (Innermost servo) is set to 0 degrees
		J1_ang = 180
		J2_ang = 180

		k = 1 #Joint counter
		goal_reached = False

		



kinematics = Kinematics()
servo0 = Servo_control(2) #Outer most servo
servo1 = Servo_control(3)
servo2 = Servo_control(4) #Inner most servo

while True:
    kinematics.play_forward()
