# many of these functions are explained at: docs.ros2.org/galactic.api/rclpy/index.html

# import main ROS2 libraries
import rclpy
import time
import pandas as pd
from rclpy.node import Node
from std_msgs.msg import String
from turtlesim.srv import TeleportAbsolute
# import other dependencies
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt, pi

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


		# sub 2
		self.cam_sub = self.create_subscription(Pose, 'topic', self.cam_update, 10)

		# create service client
		self.client = self.create_client(TeleportAbsolute, 'turtle1/teleport_absolute')
		self.request = TeleportAbsolute.Request()

		# initialize global errors (integral accumlation)
		self.prev_error_linear = 0.0
		self.prev_error_angular = 0.0
		self.integral_error_linear = 0.0
		self.integral_error_angular = 0.0

		# member variable for position
		self.pose = Pose()
		self.cam_update_called = False
		self.cam_pose = Pose()
		# establish rate
		self.rate = self.create_rate(10)
	
	def cam_update(self, data):
		self.cam_pose = data
		self.cam_update_called = True		
		#print(self.cam_pose.x, self.cam_pose.y)

	# resets turtle position
	def send_request(self, x, y, theta):
		self.request.x = x
		self.request.y = y
		self.request.theta = theta
		self.future = self.client.call_async(self.request)
		rclpy.spin_until_future_complete(self, self.future)
		return self.future.result()

	def update_pose(self, pose: Pose):
		# this is the callback function which is called when a new message of type pose is recieved by subscriber
		self.pose = pose
		self.pose.x = round(self.pose.x, 4)
		self.pose.y = round(self.pose.y, 4)

	def euclidean_distance(self, goal_pose):
		# used to compute euclidean distance between current pose (position) and goal.
		rclpy.spin_once(self)
		return sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2))

	def steering_angle(self, goal_pose):
		rclpy.spin_once(self)
		return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

	def align_angle(self, goal_pose, angular_tolerance, ctrl_type, kpa, kia, kda):
		msg = Twist()
		initial_time = self.get_clock().now().nanoseconds
		prev_time = self.get_clock().now().nanoseconds
		firstIteration = True
		correct_direction = True
		close = False
		oscillations = 0
		#print(f"kpa: {kpa}, kia: {kia}, kda: {kda}")
		while abs(self.steering_angle(goal_pose) - self.pose.theta) >= angular_tolerance:
			# check for messages received by subsriber to allow for callback function calls
			rclpy.spin_once(self)

			# if spinning out of control, return fail
			if (abs(self.steering_angle(goal_pose) - self.pose.theta)) <= angular_tolerance * 2:
				close = True
			if (close) and (abs(self.steering_angle(goal_pose) - self.pose.theta)) >= angular_tolerance * 2:
				return "Fail", "Fail", oscillations

			# calculate angular error
			angular_error = self.steering_angle(goal_pose) - self.pose.theta
			#print(f"Ang err: {angular_error}, Ang prev: {self.prev_error_angular}")
			if (abs(angular_error) > abs((self.prev_error_angular)) and correct_direction and not firstIteration):
				print("CHANGE DIRECTION")
				oscillations += 1 # increment oscillations by 1
				correct_direction = False
			# if correct_direction is false, it means the robot has already changed direction. this means we must check for if
			# the prev error is > than the current error. 
			elif (angular_error < (self.prev_error_angular) and correct_direction == False and not firstIteration):
				print("CHANGE DIRECTION")
				oscillations += 1
				correct_direction = True
			
			# calculate dt
			current_time = self.get_clock().now().nanoseconds
			delta_time = (current_time - prev_time) / 1e9
			
			prev_time = current_time

			# add to integral error (accumulates over time)
			self.integral_error_angular += angular_error * delta_time
			
			# on first loop iteration, dt is not valid and is arbitrarily large. dont include derivative and integral components
			if firstIteration:
				angular_derivative = 0.0
				self.integral_error_angular = 0.0
				firstIteration = False
			else:
				angular_derivative = (angular_error - self.prev_error_angular) / delta_time

			# set prev angular error
			self.prev_error_angular = angular_error

			# P controller
			if ctrl_type == "P":
				msg.angular.z = kpa * angular_error
			# PI controller
			elif ctrl_type == "PI":
				msg.angular.z = kpa * angular_error + kia * self.integral_error_angular
			# PID controller
			elif ctrl_type == "PID":
				msg.angular.z = kpa * angular_error + kda * angular_derivative + kia * self.integral_error_angular
			
			# publish
			msg.linear.x = 0.0
			self.velocity_publisher.publish(msg)
		
		# reset accumulated error back to zero (for iterative testing)
		self.integral_error_angular = 0.0
	
		# get final time
		final_time = self.get_clock().now().nanoseconds
		
		# stop rotation once angle is aligned according to angular_tolerance
		msg.angular.z = 0.0
		
		# spin one last time to make sure that self.pose is up to date
		rclpy.spin_once(self)

		#print(self.steering_angle(goal_pose))
		#print(self.pose.theta)
		
		# publish to stop rotation
		self.velocity_publisher.publish(msg)

	
		return abs((self.steering_angle(goal_pose) - self.pose.theta) * 180 / pi), ((final_time - initial_time)/1e9), oscillations

	def cam_goal(self):
		'''
		rclpy.spin_once(self)
		print(self.cam_pose.x)
		#print((self.cam_pose.x - 320) / (110/640)) 
		return (self.cam_pose.x - 320)/(110/640)
		'''	
	
	def cam_angle(self, angular_tolerance, kp, ki, kd):
		while not self.cam_update_called:
			rclpy.spin_once(self)
	
		msg = Twist()
		
		while (abs(self.cam_pose.x - 320) >= angular_tolerance):
		#while (abs(self.cam_goal() - self.pose.theta >= angular_tolerance)):
			rclpy.spin_once(self)
			print(self.cam_pose.x - 320)
			msg.angular.z = (self.cam_pose.x - 320)/(110 * 1000/640)
			self.velocity_publisher.publish(msg)	

		msg.angular.z = 0.0
		self.velocity_publisher.publish(msg)	
		

	def move2goal(self, goal_pose, distance_tolerance, ctrl_type, kp, ki, kd):
		msg = Twist()
		initial_time = self.get_clock().now().nanoseconds
		prev_time = self.get_clock().now().nanoseconds
		
		# bool to determine if first loop
		firstIteration = True
		close = False

		# the turtle is assumed to initially be headed in the 'correct' direction
		correct_direction = True

		# initialize oscillations variable to zero
		oscillations = 0
		
		# check for updates to pose
		rclpy.spin_once(self)

		# while distance from self to target is greater than set tolerance
		while self.euclidean_distance(goal_pose) >= distance_tolerance:
		# spin to check for any messages received by subscriber.
			rclpy.spin_once(self)
			
			# if turtle reaches borders of screen, return fail (assuming goal is not on border)
			if (self.pose.x <= 0.0 or self.pose.y <= 0.0) or (self.pose.x >= 11.0 or self.pose.y >= 11.0):
				self.integral_error_linear = 0.0
				return "Fail", "Fail", oscillations	
			
			# compute linear error
			linear_error = self.euclidean_distance(goal_pose)
			#print(f"LIN ERR: {linear_error}, PREV ERR: {self.prev_error_linear}")
			
			# the below if statements checks if the turtle has changed direction. if the current error is > then previous error, it has
			# reversed directions. The previous error is always supposed to be greater than the current error.
			if (linear_error > (self.prev_error_linear + 0.2) and correct_direction and not firstIteration):
			#	print("CHANGE DIRECTION")
				oscillations += 1 # increment oscillations by 1
				correct_direction = False
			# if correct_direction is false, it means the robot has already changed direction. this means we must check for if
			# the prev error is > than the current error. 
			elif (linear_error < (self.prev_error_linear - 0.2) and correct_direction == False and not firstIteration):
			#	print("CHANGE DIRECTION")
				oscillations += 1
				correct_direction = True

			# calculate dt
			current_time = self.get_clock().now().nanoseconds
			delta_time = (current_time - prev_time) / 1e9
			prev_time = current_time

			# add to integral error (accumulates over time)
			self.integral_error_linear += linear_error * delta_time
			
			# first loop yields interesting results ->  invalid dt
			if firstIteration:
				linear_derivative = 0.0
				self.integral_error_linear = 0.0
				firstIteration = False
			else:
				linear_derivative = (linear_error - self.prev_error_linear) / delta_time
			
			# set prev error linear
			self.prev_error_linear = linear_error
			
			# controller selection 
			if ctrl_type == "P":
				msg.linear.x = kp * linear_error
			elif ctrl_type == "PI":
				msg.linear.x = kp * linear_error + ki * self.integral_error_linear
			elif ctrl_type == "PID":
				msg.linear.x = kp * linear_error + kd * linear_derivative + ki * self.integral_error_linear

			msg.linear.y = 0.0
			msg.linear.z = 0.0
			self.velocity_publisher.publish(msg)

		# reset accumulated error back to zero.
		self.integral_error_linear = 0.0

		# stop movement once close enough to target
		final_time = self.get_clock().now().nanoseconds

		# check for any pos updates
		rclpy.spin_once(self)

		# stop the robot
		msg.linear.x = 0.0
		self.velocity_publisher.publish(msg)

		return self.euclidean_distance(goal_pose), (final_time - initial_time)/1e9, oscillations

	def gtg(self, goal_pose, distance_tolerance, ctrl_type, kp, ki, kd, kpa, kia, kda, bool):
		msg = Twist()
		# new addition
		#prev_time = self.get_clock().now().seconds_nanoseconds()[0]
		initial_time = self.get_clock().now().nanoseconds
		prev_time = self.get_clock().now().nanoseconds

		initial_dist = self.euclidean_distance(goal_pose)
		firstIteration = True		
		startloop = False
		rclpy.spin_once(self)
		# generate angular velocities
		while (self.euclidean_distance(goal_pose) >= distance_tolerance):
			rclpy.spin_once(self)

			if (self.euclidean_distance(goal_pose) <= 0.3 * initial_dist) and bool == True:
				goal_pose = self.change_goal()

			linear_error = self.euclidean_distance(goal_pose)
			angular_error = self.steering_angle(goal_pose) - self.pose.theta

			current_time = self.get_clock().now().nanoseconds

			
			delta_time = (current_time - prev_time)/ 1e9
			prev_time = current_time
			print(f"DELTATIME: {delta_time}")
			self.integral_error_linear += linear_error * delta_time 
			self.integral_error_angular += angular_error * delta_time
			
			if firstIteration:
				linear_derivative = 0.0
				angular_derivative = 0.0
			else:
				linear_derivative = (linear_error - self.prev_error_linear) / delta_time
				angular_derivative = (angular_error - self.prev_error_angular) / delta_time

			# Proportional controller
			if ctrl_type == "P":
				msg.linear.x = kp * linear_error
				msg.angular.z = kpa * angular_error

			# Proporitonal, Integral controller
			elif ctrl_type == "PI":
				msg.linear.x = kp * linear_error + ki * self.integral_error_linear
				msg.angular.z = kpa * angular_error + kia * self.integral_error_angular
			
			# Proporitonal, Integral, and Derivative PID
			elif ctrl_type == "PID":
				msg.linear.x = kp * linear_error + kd * linear_derivative + ki * self.integral_error_linear
				msg.angular.z = kpa * angular_error + kda * angular_derivative + kia * self.integral_error_angular



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

	def change_goal(self):
		new_goal = Pose()
		new_goal.x = 8.0
		new_goal.y = 8.0
		return new_goal

	def main_loop(self):

		#op = int(input("Simultaneous or sequential motor control?\n1) Simultaneous\n2) Sequential\n"))
		goal_pose = Pose()
		op1 =int(input("\nChoose mode of operation:\n\t1) Angular gain optimizer\n\t2) Linear gain optimizer\n\t3) Regular (Angular -> Linear movement)\n\t4) Camera Input \n\n>> "))
		# if angular gain optimizer
		if op1 == 1:
			# get inputs from user
			goal_pose.x = float(input("Set your x goal: "))
			goal_pose.y = float(input("Set your y goal: "))
			ctrl_type = input("\nChoose control type:\n	P\n	PI\n	PID\n>> ")		
			angular_tolerance = float(input("\nSet angular tolerance: "))

			# construct data frame
			columns = ['kpa', 'kia', 'kda', 'angle_error', 'angle_time', 'oscillations']
			df1 = pd.DataFrame(columns=columns)
			rows=[]

			# loop to calculate optimal angular gain parameters
			for kpa in range(1,16,2):
				for kia in range (1,16,2):
					for kda in range (1,16,2):
						print(f"kpa: {kpa/10}, kia: {kia/10}, kda: {kda/10}")
						angle_error, angle_time, oscillations = self.align_angle(goal_pose, angular_tolerance, ctrl_type, kpa/10, kia/10, kda/10)
						print(f"Angle error: {angle_error}, Angle time: {angle_time}, Oscillations: {oscillations}")
						
						self.send_request(5.544445, 5.544445, 0.0)
						time.sleep(1)
						row = {'kpa': kpa/10, 'kia': kia/10, 'kda': kda/10, 'angle_error': angle_error, 'angle_time': angle_time,
							 'oscillations': oscillations}
						rows.append(row)
			df1 = pd.concat([pd.DataFrame([row]) for row in rows],  ignore_index=True)
			
			# output csv
			df1.to_csv('pid_tuning_results_angular.csv', index=False)
		
		# linear gain optimizer
		elif op1 == 2:
			# get input from user
			goal_pose.x = float(input("Set your x goal: "))
			goal_pose.y = float(input("Set your y goal: "))
			ctrl_type = input("\nChoose control type:\n	P\n	PI\n	PID\n>> ")		
			distance_tolerance = float(input("\nSet distance tolerance: "))
			
			# initialize angle to whatever
			self.send_request(5.544445, 5.544445, 225 * pi / 180)

			# construct data frame
			columns = ['kp', 'ki', 'kd', 'linear_error', 'linear_time, oscillations']
			df2 = pd.DataFrame(columns=columns)
			rows2=[]

			# loop through different gain values
			for kp in range(1,16,2):
				for ki in range (1,16,2):
					for kd in range (1,16,2):
						print(f"kp: {kp/10}, ki: {ki/10}, kd: {kd/10}")
						linear_error, linear_time, num_oscillations = self.move2goal(goal_pose, distance_tolerance,  ctrl_type, kp/10, ki/10, kd/10)
						time.sleep(1)
						print(f"Linear error: {linear_error}, Linear time: {linear_time}, oscillations: {num_oscillations}")
						self.send_request(5.544445, 5.544445, 225 * pi /180)
						time.sleep(2)
						row = {'kp': kp/10, 'ki': ki/10, 'kd': kd/10, 'linear_error': linear_error, 'linear_time': linear_time, 'oscillations':
							 num_oscillations}
						rows2.append(row)

			# output csv
			df2 = pd.concat([pd.DataFrame([row]) for row in rows2],  ignore_index=True)
			df2.to_csv('pid_tuning_results_linear.csv', index=False)

		# if regular mode (angular then linear movement)
		elif op1 == 3:
			# get inpt from user
			op2 = input("\nMulti-waypoint? (y/n)\n>> ")
			op3 = int(input("\nSimultaneous or sequential motor control?\n\t1) Simultaneous\n\t2) Sequential\n\n>> "))		
			ctrl_type = input("\nChoose control type:\n	P\n	PI\n	PID\n>> ")		
			kp = float(input("\nKp: "))
			ki = float(input("Ki: "))
			kd = float(input("Kd: "))
			kpa = float(input("Kpa: "))
			kia = float(input("Kia: "))
			kda = float(input("Kda: "))

			distance_tolerance = float(input("\nSet distance tolerance: "))
			angular_tolerance = float(input("Set angular tolerance: "))

			list_waypoints = []
				
			if op2 == "y":
				num_waypoints = int(input("\nEnter number of way points: "))
				for i in range (0, num_waypoints):
					print(f"\nWaypoint {i+1}")
					goal_pose.x = float(input("Set your x goal: "))
					goal_pose.y = float(input("Set your y goal: "))
					pos_tuple = (goal_pose.x, goal_pose.y)
					list_waypoints.append(pos_tuple)
				if op3 == 1:
					for i in range (0, num_waypoints):
						pos_tuple = list_waypoints[i]
						print(pos_tuple)
						goal_pose.x = pos_tuple[0]
						goal_pose.y = pos_tuple[1]
						self.gtg(goal_pose, distance_tolerance, ctrl_type, kp, ki, kd, kpa, kia, kda, bool)
						time.sleep(1)

				elif op3 == 2:
					for i in range (0, num_waypoints):
						pos_tuple = list_waypoints[i]
						goal_pose.x = pos_tuple[0]
						goal_pose.y = pos_tuple[1]
						print(f"\nMovement {i+1}")
						angle_error, angle_time, oscillations = self.align_angle(goal_pose, angular_tolerance, ctrl_type, kpa, kia, kda)
						print(f"Angle error: {angle_error}, Angle time: {angle_time}, Oscillations: {oscillations}")
						# print(self.pose.x, self.pose.y, self.pose.theta)						
						linear_error, linear_time, oscillations = self.move2goal(goal_pose, distance_tolerance, ctrl_type, kp, ki, kd)
						print(f"Linear error: {linear_error}, Linear Time: {linear_time}, Oscillations: {oscillations}")

						#print(self.pose.x, self.pose.y, self.pose.theta)		
						#self.send_request(5.544445, 5.544445, 0.0)
						time.sleep(1)
	
			else:
				# get input from users
				goal_pose.x = float(input("Set your x goal: "))
				goal_pose.y = float(input("Set your y goal: "))
				
				if op3 == 1:
					self.gtg(goal_pose, distance_tolerance, ctrl_type, kp, ki, kd, kpa, kia, kda, bool)

				elif op3 == 2:
					angle_error, angle_time, oscillations = self.align_angle(goal_pose, angular_tolerance, ctrl_type, kpa, kia, kda)
					linear_error, linear_time, oscillations = self.move2goal(goal_pose, distance_tolerance, ctrl_type, kp, ki, kd)	
					print(f"Angle error: {angle_error}, Angle time: {angle_time}, oscillations: {oscillations}")
					print(f"Linear error: {linear_error}, Linear time: {linear_time}, oscillations: {oscillations}")
		elif op1 == 4:
			ang_tol = float(input("Angulkar tolerance: "))
			self.cam_angle(ang_tol, 0, 0, 0)
def main(args=None):

	rclpy.init(args=args)
	x = TurtleBot()
	rclpy.spin_once(x)
	x.main_loop()
	x.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()


