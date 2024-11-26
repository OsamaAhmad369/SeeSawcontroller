#!/usr/bin/env python
import time

#--------------------------------------------------------------------------------------------------------------------------------------

def PID(pitch, f):
	#Define the global variables to prevent them from dying and resetting to zero, each time a function call occurs. Some of these variables 		may be redundant.
	global kp_roll, ki_roll, kd_roll, kp_pitch, ki_pitch, kd_pitch, kp_yaw, ki_yaw, kd_yaw, prevErr_roll, prevErr_pitch, prevErr_yaw, pMem_roll, pMem_yaw, pMem_pitch, iMem_roll, iMem_pitch, iMem_yaw, dMem_roll, dMem_pitch, dMem_yaw, flag, setpoint, sampleTime
	#-----------------------
	#Assign your PID values here. From symmetry, control for roll and pitch is the same.
	kp_roll = 68
	ki_roll = 0.001
	kd_roll = 88
	kp_pitch = kp_roll
	ki_pitch = ki_roll
	kd_pitch = kd_roll
	kp_yaw = 0.1
	ki_yaw = 0
	kd_yaw = 0
	flag = 0
	#Define other variables here, and calculate the errors.
	sampleTime = 0
	setpoint = 0
	err_pitch = float(pitch)*(180 / 3.141592653) - setpoint 
	currTime = time.time()
	#-----------------------
	#Reset the following variables during the first run only.
	if flag == 0:
		prevTime = 0
		prevErr_roll = 0
		prevErr_pitch = 0
		prevErr_yaw = 0
		pMem_roll = 0
		pMem_pitch = 0
		pMem_yaw = 0
		iMem_roll = 0
		iMem_pitch = 0
		iMem_yaw = 0
		dMem_roll = 0
		dMem_pitch = 0
		dMem_yaw = 0
		flag += 1
	#------------------------
	#Define dt, dy(t) here for kd calculations.
	dTime = currTime - prevTime
	dErr_pitch = err_pitch - prevErr_pitch
	
	#-------------------------------------------------------------------------------------------------------------------------------
	#This is the Heart of the PID algorithm. PID behaves more accurately, if it is sampled at regular intervals. You can change the sampleTime to whatever value is suitable for your plant.
	if(dTime >= sampleTime):
		#Kp*e(t)

		pMem_pitch = kp_pitch * err_pitch
	
		
		#integral(e(t))
		iMem_pitch += err_pitch * dTime
		
		if(iMem_pitch > 400): iMem_pitch = 400
		if(iMem_pitch < -400): iMem_pitch = -400
		
		#derivative(e(t))
		dMem_pitch = dErr_pitch / dTime	
	#Store the current variables into previous variables for the next iteration.
	prevTime = currTime
	prevErr_pitch = err_pitch
	
	#output = Kp*e(t) + Ki*integral(e(t)) + Kd*derivative(e(t))
	output_pitch = pMem_pitch + ki_pitch * iMem_pitch + kd_pitch * dMem_pitch
	
	#fl in my code is bl in gazebo's world
	esc_fl = 1500  - output_pitch 
	#fr in my code is fl in gazebo's world
	esc_fr = 1500 + output_pitch 
	#Limit the ESC pulses to upper limit and lower limit, in case the PID algorithm goes crazy and high af.
	
	if(esc_fr > 2000): esc_fr = 2000
	if(esc_fl > 2000): esc_fl = 2000
	
	
	if(esc_fr < 1100): esc_fr = 1100
	if(esc_fl < 1100): esc_fl = 1100
	
	#Map the esc values to motor values
	
	fr_motor_vel = ((esc_fr - 1500)/25) + 50
	fl_motor_vel = ((esc_fl - 1500)/25) + 50
	#----------------------------------------------------------------------------------------------------------------------------------
	
	#---------------------------------------------------------------------------------------------------------------------------------
	#Provide the motor velocities to the object 'f' that will now exit out of this function, and gets published to gazebo, providing velocities to each motor. Note that the sign here is +,-,+,- i.e CW, CCW, CW, CCW in gazebo model. Change view of gazebo model (by scrolling) such that the green line comes to your left, red line goes forward, and blue line goes upward. This is the convention that i refer to as "Gazebo model" incase you get confused.
	f.data = [fr_motor_vel,fl_motor_vel]
	
	#Return these variables back to the control file.
	return f, err_pitch
#--------------------------------------------------------------------------------------------------------------------------------------




