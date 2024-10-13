import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class CamPub(Node):
	def __init__(self):
		super().__init__('cam_pub')

		# create publisher that publishes center value to topic
		self.cam_pub = self.create_publisher(Pose, '/topic', 10)
		self.create_rate(10)


def main(args=None):
	rclpy.init(args=args)
	camNode = CamPub()
	rclpy.spin_once(camNode)
	print('Hi from cam_pkg.')
	# construct node
		
	cap = cv2.VideoCapture(0)
	print(int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)))
	#lower_blue = np.array([198, 100, 43.9])	
	#higher_blue = np.array([198, 96.1, 100])
	lower_blue = np.array([100, 150, 0])
	upper_blue = np.array([140, 255, 255])
	goal_pose = Pose()
	while True:
		ret, frame = cap.read()
		if not ret:
			print("Error: could not read frame")
			break
	
		hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	
		mask = cv2.inRange(hsv, lower_blue, upper_blue)
		cv2.imshow('Blue mask', mask)
		contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		
		if contours:
			largest_contour = max(contours, key=cv2.contourArea)

		#for contour in contours:
			if cv2.contourArea(largest_contour) > 500:
				x, y, w, h = cv2.boundingRect(largest_contour)
				cv2.rectangle(frame, (x,y), (x + w, y + h), (0, 255, 0), 2)
				
				center_x = x + w // 2
				center_y = y + h // 2
				goal_pose.x = float(center_x)
				goal_pose.y = float(center_y)

				camNode.cam_pub.publish(goal_pose)

				cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)

				text = f"Center: {center_x}, {center_y}"
				cv2.putText(frame, text, (center_x - 50, center_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

				
				text2 = f"X: {x}, y: {y}"
				cv2.putText(frame, text2, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
				



		cv2.imshow('Detected objects', frame)

		if cv2.waitKey(1) & 0xFF == ord('q'):
			break

	cap.release()
	cv2.destoryAllWindows()
	camNode.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
    	main()
