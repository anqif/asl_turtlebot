#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from supervisor import POSE_EPS, THETA_EPS
from utils import state_to_pose
import tf
from enum import Enum

class Mode(Enum):
    START = 1
    COMM = 2
    LEAD = 3
    RESCUE = 4
    IDLE = 5

class Rescuer:
	
	def __init__(self):
		rospy.init_node('turtlebot_rescuer', anonymous=True)

		# current pose
        self.x = 0
        self.y = 0
        self.theta = 0

        # start pose
        self.x_s = 0
        self.y_s = 0
        self.theta_s = 0

        # current mode
        self.mode = Mode.IDLE
        self.last_mode_printed = None

        # animals to rescue
		self.animals = []   # list of dicts {"name": str, location": (x,y,theta)}
		self.cur_animal_idx = 0   # index of current animal to rescue

		self.move_goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
		self.rescue_rdy_publisher = rospy.Publisher('/ready_to_rescue', bool, queue_size=10)

		rospy.Subscriber('/rescue_on', bool, self.rescue_on_callback)
		
		self.trans_listener = tf.TransformListener()

	def rescue_on_callback(self, msg):
		""" callback for signal to start rescue """

		self.mode = Mode.START if msg else Mode.IDLE   # TODO: Can we assume /rescue_on publishes True throughout rescue operation?

	def go_to_start(self):
		""" sends the starting pose to the supervisor """

		pose = state_to_pose(self.x_s, self.y_s, self.theta_s)
		self.move_goal_publisher.publish(pose)

	def go_to_animal(self, idx = None):
		""" sends the animal location to the supervisor """

		if idx is None:
			idx = self.cur_animal_idx
		x_a, y_a, theta_a = self.animals[idx].location
		pose = state_to_pose(x_a, y_a, theta_a)
		self.move_goal_publisher.publish(pose)

	def cur_animal(self):
		""" returns current animal to rescue """

		return self.animals[self.cur_animal_idx]

	def next_animal(self):
		""" increments counter to next animal to rescue; returns False if none left """

		if self.cur_animal_idx < (len(self.animals) - 1):
			self.cur_animal_idx += 1
			return True
		else:
			return False

	def close_to(self, x, y, theta):
        """ checks if the robot is at a pose within some threshold """

        return (abs(x-self.x) < POS_EPS and abs(y-self.y) < POS_EPS and abs(theta-self.theta) < THETA_EPS)

	def loop(self):
		# Get current position on map
		try:
            (translation,rotation) = self.trans_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
            self.x = translation[0]
            self.y = translation[1]
            euler = tf.transformations.euler_from_quaternion(rotation)
            self.theta = euler[2]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

        # Logs the current mode
        if not(self.last_mode_printed == self.mode):
            rospy.loginfo("Current Mode: %s", self.mode)
            self.last_mode_printed = self.mode

        if self.mode == Mode.START:   # Go to start position
        	if self.close_to(self.x_s, self.y_s, self.theta_s):
        		self.mode = Mode.COMM
        	else:
				self.go_to_start()

		elif self.mode == Mode.COMM:   # Communicate with rescuers
			if self.start_rescue:
				self.mode = Mode.LEAD
			else:
				self.rescue_rdy_publisher.publish(True)

		elif self.mode == Mode.LEAD:   # Return to current animal
			if self.close_to(*self.cur_animal().location):
				self.mode = Mode.LEAD
			else:
				self.go_to_animal()

		elif self.mode == Mode.RESCUE:   # Rescue current animal and increment to next if available
			rospy.loginfo("Rescuing Animal: %s", self.cur_animal().name)
			if self.next_animal():
				self.mode = Mode.LEAD
			else:
				self.mode = Mode.IDLE

		elif self.mode == Mode.IDLE:
			pass

		else:
			raise Exception("This mode is not supported: %s" % str(self.mode))

	def run(self):
		rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()

if __name__ == '__main__':
    res = Rescuer()
    res.run()