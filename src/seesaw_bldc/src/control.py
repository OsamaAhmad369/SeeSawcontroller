#!/usr/bin/env python
#---------------------------------------------------
''''
Multiprocessing has been implemented to measure imu readings and apply control scheme 

'''
import numpy as np
from pid import PID
import rospy
from multiprocessing import Process, Manager
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float64MultiArray, Float32
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import Imu
#---------------------------------------------------
f = Float64MultiArray()

def imu_listener_node(euler_angles):
	def imu_callback(data):
		# Extract orientation from the IMU data
		orientation_q = data.orientation #data.angular_velocity, data.linear_acceleration 
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		velocity=data.angular_velocity
		# Convert quaternion to Euler angles (roll, pitch, yaw)
		(roll, pitch, yaw) = euler_from_quaternion(orientation_list)
		euler_angles['pitch']=pitch
		euler_angles['velocity']=velocity.y
		# Display the angles
		rospy.loginfo("Roll: {:.2f}, Pitch: {:.2f}, Yaw: {:.2f}".format(roll, pitch, yaw))
	rospy.init_node('imu_listener', anonymous=True)
	# Subscribe to the IMU data topic (update topic name as per the configuration)
	rospy.Subscriber("/seesaw/imu", Imu, imu_callback)
	rospy.spin()

def control_seesaw_node(euler_angles):

	def control_seesaw(msg, args):
		#Declare global variables as you dont want these to die, reset to zero and then re-initiate when the function is called again.
		global pitch, err_pitch,fUpdated,state
		pitch=euler_angles['pitch']
		err_pitch=0
		(fUpdated, err_pitch) = PID(pitch, f)
		
		args[0].publish(fUpdated)
		args[1].publish(err_pitch)
		

	rospy.init_node("Control")

	#initiate publishers that publish errors (roll, pitch,yaw - setpoint) so that it can be plotted via rqt_plot /err_<name>  
	err_pitchPub = rospy.Publisher('err_pitch', Float32, queue_size=1)

	#initialte publisher velPub that will publish the velocities of individual BLDC motors

	velPub = rospy.Publisher('/seesaw/joint_motor_controller/command', Float64MultiArray, queue_size=2)

	#Subscribe to /gazebo/model_states to obtain the pose in quaternion form
	#Upon receiveing the messages, the objects msg, velPub, err_rollPub, err_pitchPub and err_yawPub are sent to "control_seesaw" function.
	PoseSub = rospy.Subscriber('/gazebo/model_states',ModelStates,control_seesaw,(velPub, err_pitchPub))
	rospy.spin()
if __name__ == '__main__':
	try:
		manager = Manager()
		euler_angles = manager.dict()
		euler_angles['pitch']=0.0
		imu_process = Process(target=imu_listener_node,args=(euler_angles,))
		control_process = Process(target=control_seesaw_node,args=(euler_angles,))
		imu_process.start()
		control_process.start()
		imu_process.join()
		control_process.join()
	except rospy.ROSInterruptException:
		pass