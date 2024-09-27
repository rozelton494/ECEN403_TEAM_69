# many of these functions are explained at: docs.ros2.org/galactic.api/rclpy/index.html

# import main ROS2 libraries
import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import String
from turtlesim.srv import Spawn
# import other dependencies
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt

# class TurtleBot which inherits from Node
class TurtleBot(Node):
	def __init__(self):
		# name node upon construction
		super().__init__('turtlebot_controller')

		# create publisher that publishes to the topic '/turtle1/cmd_vel'.
		# message is of type Twist and QoS given as 10 (default?)
		self.velocity_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

		# create subscriber to the topic '/turtle1/cmd_vel' method called self.update_pose is called when message received.
		# messeage is of type pose, subscribes to the topic /turtle1/pose and the callback function is called update_pose. call back function is called when message is received.
		self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.update_pose, 10)

		# next addition
		self.kp = 0.5
		self.kpa = 2
		self.ki = 0.1
		self.kd = 0.1
		self.prev_error_linear = 0.0
		self.prev_error_angular = 0.0
		self.integral_error_linear = 0.0
		self.integral_error_angular = 0.0


		# member variable for position
		self.pose = Pose()
		#self.pose_received = False
		self.rate = self.create_rate(10)

	#def wait_for_pose(self):
	#	while not self.pose_received:
	#		rclpy.spin_once(self)
	#		self.rate.sleep()
	#	print("POSE RECEIVED!!!!")
	
	def update_pose(self, pose: Pose):
		# this is the callback function which is called when a new message of type pose is recieved by subscriber
		#print('message received by subscriber')
		#print(data)
		#print("Callback function called")
		self.pose = pose
		self.pose.x = round(self.pose.x, 4)
		self.pose.y = round(self.pose.y, 4)
		#print("END OF CALLBACK")
		#self.pose_received = True
		#print("pose_received -> True")

	def euclidean_distance(self, goal_pose):
		# used to compute euclidean distance between current pose (position) and goal.
		print (sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2)))
		return sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2))

	def linear_vel(self, goal_pose, constant = 0.5):
		# relates to proportional control system
		return constant *  self.euclidean_distance(goal_pose)

	def steering_angle(self, goal_pose):
		print("STEERING ANGLE CALCULATION")
		#print(f"x: {self.pose.x}, y: {self.pose.y}")
		#print(f"x: {goal_pose.x}, y: {goal_pose.y}")
		#print(f"theta: {self.pose.theta}")
		print(f"ATAN: {atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)}")
		return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

	def angular_vel(self, goal_pose, constant=2):
		return constant * (self.steering_angle(goal_pose) - self.pose.theta)
	
	def align_angle(self, goal_pose):
		twist = Twist()
		angular_tolerance = 0.01 # rad
		#while self.pose.theta == 0:
		#	rclpy.spin_once(self)
		#count = 10
		while abs(self.steering_angle(goal_pose) - self.pose.theta) >= angular_tolerance:
			#print("TEST")
			#print(self.pose.theta)
			#print(self.steering_angle(goal_pose) - self.pose.theta) 
			rclpy.spin_once(self)
			twist.linear.x = 0.0
			twist.angular.z = float(self.angular_vel(goal_pose))

			self.velocity_publisher.publish(twist)
			#print(self.pose.theta)

			#self.rate.sleep()

		if (self.steering_angle(goal_pose) - self.pose.theta) <=  angular_tolerance:
			print(f"THETA: {self.pose.theta}, STEERANGLE: {self.steering_angle(goal_pose)}")
			#print(f"SELF STEERING ANGLE: {self.steering_angle(goal_pose) - self.pose.theta} <= {angular_tolerance}")
			
		

		# stop rotation once angle is aligned
		twist.angular.z = 0.0
		self.velocity_publisher.publish(twist)
		print("Done rotating.")
		#rclpy.spin(self)

	def move2goal(self, goal_pose, distance_tolerance):

		vel_msg = Twist()


		# phase 1 of movement
		while self.euclidean_distance(goal_pose) >= distance_tolerance:
		# proportional controller
			rclpy.spin_once(self)

			vel_msg.linear.x = float(self.linear_vel(goal_pose))
			vel_msg.linear.y = 0.0
			vel_msg.linear.z = 0.0
			self.velocity_publisher.publish(vel_msg)

		# stop robot after movement is over
		vel_msg.linear.x = 0.0
		self.velocity_publisher.publish(vel_msg)

	def gtg(self, goal_pose, distance_tolerance, ctrl_type):
		msg = Twist()
		# new addition
		#prev_time = self.get_clock().now().seconds_nanoseconds()[0]
		initial_time = self.get_clock().now().nanoseconds
		prev_time = self.get_clock().now().nanoseconds
		
		firstIteration = True		
		startloop = False
		# generate angular velocities
		while (self.euclidean_distance(goal_pose) >= distance_tolerance):
			rclpy.spin_once(self)
			#if startloop == False:
			#	prev_time = self.get_clock().now().nanoseconds
			#	startloop = True

			# new addiiton
			linear_error = self.euclidean_distance(goal_pose)
			angular_error = self.steering_angle(goal_pose) - self.pose.theta

			#print(f"TESTING DERIVATIVE: linearERR: {linear_error}, prevlinERR: {self.prev_error_linear}\nAngERR: {angular_error}, prevAngERR: {self.prev_error_angular}")


			current_time = self.get_clock().now().nanoseconds

			
			delta_time = (current_time - prev_time)/ 1e9
			prev_time = current_time
			print(f"DELTATIME: {delta_time}")
			if delta_time == 0 or delta_time < 1e-6:
				delta_time = 1e-6
			self.integral_error_linear += linear_error * delta_time
			self.integral_error_angular += angular_error * delta_time
			
			if firstIteration:
				linear_derivative = 0.0
				angular_derivative = 0.0
			else:
				linear_derivative = (linear_error - self.prev_error_linear) / delta_time
				angular_derivative = (angular_error - self.prev_error_angular) / delta_time
			
			#print(f"LIN DERIV: {linear_derivative}, ANG DERIV: {angular_derivative}")
			#msg.linear.x = (self.kp * linear_error +
			#		self.ki * self.integral_error_angular +
			#		self.kd * angular_derivative)

			#msg.angular.z = (self.kp * angular_error +
			#		self.ki * self.integral_error_angular +
			#		self.kd * angular_derivative)

			# Proportional controller
			if ctrl_type == "P":
				msg.linear.x = self.kp * linear_error
				msg.angular.z = self.kpa * angular_error

			# Proporitonal, Integral controller
			elif ctrl_type == "PI":
				msg.linear.x = self.kp * linear_error + self.ki * self.integral_error_linear
				msg.angular.z = self.kpa * angular_error + self.ki * self.integral_error_angular
			
			# Proporitonal, Integral, and Derivative PID
			elif ctrl_type == "PID":
				msg.linear.x = self.kp * linear_error + self.kd * linear_derivative + self.ki * self.integral_error_linear
				msg.angular.z = self.kpa * angular_error + self.kd * angular_derivative + self.ki * self.integral_error_angular

			# Proportional, Derivative controller
			elif ctrl_type == "PD":
				msg.linear.x = self.kp * linear_error + self.kd * linear_derivative
				msg.angular.z = self.kpa * angular_error + self.kd * angular_derivative
			
			# Integral, Derivative
			elif ctrl_type == "ID":
				msg.linear.x = self.ki * self.integral_error_linear + self.kd * linear_derivative
				msg.angular.z = self.ki * self.integral_error_angular + self.kd * angular_derivative

			self.velocity_publisher.publish(msg)
			self.prev_error_linear = linear_error
			self.prev_error_angular = angular_error
			
			
			'''

			msg.linear.x = float(self.linear_vel(goal_pose))
			msg.linear.y = 0.0
			msg.linear.z = 0.0
			
			msg.angular.x = 0.0
			msg.angular.y = 0.0
			msg.angular.z = float(self.angular_vel(goal_pose))
		
			# publish to cmd topic
			self.velocity_publisher.publish(msg)
			'''
		final_time = self.get_clock().now().nanoseconds
		final_x = self.pose.x
		final_y = self.pose.y

		print(f"Time to complete: {(final_time - initial_time) / 1e9}")
		print(f"Final position: ({final_x}, {final_y})")
		msg.linear.x = 0.0
		msg.angular.z = 0.0
		self.velocity_publisher.publish(msg)
	
	def main_loop(self):

	
		op = int(input("Simultaneous or sequential motor control?\n1) Simultaneous\n2) Sequential\n"))
		goal_pose = Pose()
		goal_pose.x = float(input("Set your x goal: "))
		goal_pose.y = float(input("Set your y goal: "))
		distance_tolerance = float(input("Set your tolerance: "))
		
		if op == 1:
			ctrl_type = input("Choose control type:\n	P\n	PI\n	PID\n	PD\n	ID\n")
			self.gtg(goal_pose, distance_tolerance, ctrl_type)
		elif op == 2:
			self.align_angle(goal_pose)
			self.move2goal(goal_pose, distance_tolerance)
		else:
			print("Invalid input")	

def main(args=None):

	rclpy.init(args=args)
	print("constructing turtle object!")
	x = TurtleBot()
	print(" moving to goal!")
	x.main_loop()
	x.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()


