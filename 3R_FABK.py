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

		self.currentServoPos = 0 #Set servo initial position to be 0 / 90 degrees
		self.currentServoPos_deg = 0
		self.servo.value = self.currentServoPos

	def set_position(self, deg): #Param. deg = Degree position to set the servo (0 - 180 degrees)
		#Turn degree position into a value in the range between -1 and 1. 0 = -1 , 1 = 180 , 0 = 90 , etc.
		pos = (deg/90) - 1 
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

		self.currentServoPos_deg = (self.currentServoPos + 1) * 90

class Kinematics: #Kinematics class
	#All distances are in millimeters (mm)
	def __init__(self):
		#Initialize link lengths
		self.a1 = 86.80 
		self.a2 = 86.80
		self.a3 = 65.05

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
		x_pos = 0
		y_pos = 0
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

		return x_pos+31.6, y_pos  # (+31.6) is to account for the fact that the first joint is technically not at (0,0) 


	def play_forward(self):
		#Get the angles to be written.
		ang0 = int(input("Joint 0 angle: "))
		ang1 = int(input("Joint 1 angle: "))
		ang2 = int(input("Joint 2 angle: "))

		#Set the servos to the specified angles
		servo0.set_position(ang0)
		servo1.set_position(ang1)
		servo2.set_position(ang2) #Write servo position. Starting from the innermost servo to reduce backlash

		#Get the position of the end-effector.
		x, y = self.get_joint_pos(3, ang0, ang1, ang2)
		print('End-effector x: ' + str(x)) #Account for distance between actual start of robot arm and the arm holder
		print('End-effector y: ' + str(y))
		print()


	#For Inverse Kinematics: Determines whether the calculated angle should be added or subtracted from a joint/servo. returns false if it should be subtracted, true otherwise
	def add_angle(self, J_num, ang, x_g, y_g):	#Params: 1. k - joint number
										#        2. Angle that the servo should be changed by
										#        3, 4. Goal point x and y
		#Calculate the slope between the joint and the goal point
			#Get the joint's position
		x_j, y_j = self.get_joint_pos(J_num, servo0.currentServoPos_deg, servo1.currentServoPos_deg, servo2.currentServoPos_deg)
			#Use slope formula to calculate the slope
		comp_slope = (y_g - y_j) / (x_g - x_j)

		#Add the angle to the joint and see if the resulting slope between the end-effector and the goal point matches the comp_slope
			#Get the position of the end-effector given the you have added the angle to the kth joint
		if J_num == 0: x_e, y_e = self.get_joint_pos(3, servo0.currentServoPos_deg + ang, servo1.currentServoPos_deg, servo2.currentServoPos_deg)
		elif J_num == 1: x_e, y_e = self.get_joint_pos(3, servo0.currentServoPos_deg, servo1.currentServoPos_deg + ang, servo2.currentServoPos_deg)
		elif J_num == 2: x_e, y_e = self.get_joint_pos(3, servo0.currentServoPos_deg, servo1.currentServoPos_deg, servo2.currentServoPos_deg + ang)
			#Calculate the slope of the line between the end-effector and the goal point
		add_slope =  (y_g - y_e) / (x_g - x_e)

		#Subtract the angle to the joint and see if the resulting slope between the end-effector and the goal point matches the comp_slope
			#Get the position of the end-effector given the you have subtracted the angle to the kth joint
		if J_num == 0: x_e, y_e = self.get_joint_pos(3, servo0.currentServoPos_deg - ang, servo1.currentServoPos_deg, servo2.currentServoPos_deg)
		elif J_num == 1: x_e, y_e = self.get_joint_pos(3, servo0.currentServoPos_deg, servo1.currentServoPos_deg - ang, servo2.currentServoPos_deg)
		elif J_num == 2: x_e, y_e = self.get_joint_pos(3, servo0.currentServoPos_deg, servo1.currentServoPos_deg, servo2.currentServoPos_deg - ang)
			#Calculate the slope of the line between the end-effector and the goal point
		subtract_slope =  (y_g - y_e) / (x_g - x_e)

		#Compare the slopes, add_slope has to be within some range of the comp_slope
		if add_slope > comp_slope - 0.2 and add_slope < comp_slope + 0.2: return True
		else: return False


	def play_inverse(self): #Params: 1, 2. The x and y coordinate to place the end effector/ the goal position's x and y coordinate
   		        	  #        3. Number of iterations of the algorithm we should go through 
		x_g = int(input("Enter x: "))
		y_g = int(input("Enter y: "))
		n = int(input("Enter number of iterations: "))

		# #Declare and/or Initialize essential variables
		servo0.set_position(180) #Joint 1 (Innermost servo) is set to 0 degrees
		servo1.set_position(0)
		servo2.set_position(0)

		sleep(3)

		k = 0 #Joint counter
		iteration_counter = 0

		#Declare the vectors to be used
		v_JE = np.array([0, 0]) #vector from the current Joint to the End-effector
		v_JG = np.array([0, 0]) #vector from the current Joint to the Goal point

		for _ in range(0, n):
			iteration_counter += 1
			#print("S_0: " + str(servo0.currentServoPos_deg)); print("S_1: " + str(servo1.currentServoPos_deg)); print("S_2: " + str(servo2.currentServoPos_deg))
			#Determine the vector component values depending on the joint counter value
				#Get the end-effector's current position given the current joint angles
			x_e, y_e = self.get_joint_pos(3, servo0.currentServoPos_deg, servo1.currentServoPos_deg, servo2.currentServoPos_deg)
				#Get the kth joint's current position given the current joint angles
			x_j, y_j = self.get_joint_pos(k, servo0.currentServoPos_deg, servo1.currentServoPos_deg, servo2.currentServoPos_deg)
				#Calculate the component values for vector JE (Joint to end-effector)
			v_JE[0] = x_e - x_j
			v_JE[1] = y_e - y_j
				#Calculate the component values for vector JG (Joint to goal point)
			v_JG[0] = x_g - x_j
			v_JG[1] = y_g - y_j

			#Find the magnitude of the vectors
			mag_v_JE = np.sqrt(v_JE.dot(v_JE)) #The dot gives you the value underneath the sqrt of a magnitude formula; therefore, sqrt gives the magnitude
			mag_v_JG = np.sqrt(v_JG.dot(v_JG))

			#Use the dot product formula to calculate the angle between the two vectors
			ang = m.acos( (v_JE.dot(v_JG)) / (mag_v_JE*mag_v_JG) )
			ang_deg = (ang*180)/m.pi

			#Change the kth joint's angle by the calculated angle
			if k == 0:
				#Check whether the determined angle should be added or subtracted
				if self.add_angle(0, ang_deg, x_g, y_g) and servo0.currentServoPos_deg + ang_deg <= 180: servo0.set_position(servo0.currentServoPos_deg + ang_deg); 
				elif not self.add_angle(0, ang_deg, x_g, y_g) and servo0.currentServoPos_deg - ang_deg >= 0: servo0.set_position(servo0.currentServoPos_deg - ang_deg);
			elif k == 1:
				#Check whether the determined angle should be added or subtracted
				if self.add_angle(1, ang_deg, x_g, y_g) and servo1.currentServoPos_deg + ang_deg <= 180: servo1.set_position(servo1.currentServoPos_deg + ang_deg); 
				elif not self.add_angle(0, ang_deg, x_g, y_g) and servo1.currentServoPos_deg - ang_deg >= 0: servo1.set_position(servo1.currentServoPos_deg - ang_deg);
			elif k == 2:
				#Check whether the determined angle should be added or subtracted
				if self.add_angle(2, ang_deg, x_g, y_g) and servo2.currentServoPos_deg + ang_deg <= 180: servo2.set_position(servo2.currentServoPos_deg + ang_deg); 
				elif not self.add_angle(0, ang_deg, x_g, y_g) and servo2.currentServoPos_deg - ang_deg >= 0: servo2.set_position(servo2.currentServoPos_deg - ang_deg);
			
			#Check if the end-effector is within a reasonable range around the goal point and stop iterating if so.
				#Calculate the end-effector's current position
			x_e, y_e = self.get_joint_pos(3, servo0.currentServoPos_deg, servo1.currentServoPos_deg, servo2.currentServoPos_deg);
				#Check if the end-effector is within a region around the goal point
			if (x_e < x_g + 15 and x_e > x_g - 15) and (y_e < y_g + 15 and y_e > y_g - 15): break

			#Increment the joint counter
			k+=1
			if k == 3: k = 0
			sleep(0.25)

		x_e, y_e = self.get_joint_pos(3, servo0.currentServoPos_deg, servo1.currentServoPos_deg, servo2.currentServoPos_deg);
		print()
		print("End-effector pos: " + "x: " + str(x_e) + "  y: " + str(y_e))
		print("Goal pos: " + "x:  " + str(x_g) + "  y: " + str(y_g))
		print("Number of iterations taken: " + str(iteration_counter))


kinematics = Kinematics()
servo0 = Servo_control(2) #Outer most servo
servo1 = Servo_control(3)
servo2 = Servo_control(4) #Inner most servo

print("*************************************************************************");
while True:
	option = input("What would you like to try? Forward Kinematics - 0 or Inverse Kinematics - 1: ");
	if not option: kinematics.play_forward()
	else: kinematics.play_inverse()
