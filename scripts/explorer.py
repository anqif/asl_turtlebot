import rospy
from geometry_msgs.msg import PoseStamped
from utils import state_to_pose, pose_to_state
import tf
import math

POS_EPS = .1

THETA_EPS = .3

class Mode(Enum):
    POSE = 1
    STOP = 2
    IDLE = 3

# Defining goal pose locations starting at corner near gate starting at the garage
EXPLORE_PLAN = [(3.4, 0.4), 
				(2.4, 0.25), 
				(0.4, 0.4), 
				(0.4, 2.5), 
				(2.5, 2.5),
				(2.0, 1.6), 
				(0.4, 1.6), 
				(0.3, 2.4), 
				(3.3, 2.7), 
				(3.14, 1.6)]

class Explorer:
	
	def __init__(self):
		rospy.init_node('turtlebot_explorer', anonymous=True)
		
		# current pose
        self.x = 0
        self.y = 0
        self.theta = 0
        
        # current mode
        self.mode = Mode.IDLE
        self.last_mode_printed = None
		
		# list of goal poses for exploration
		self.expl_goals = [state_to_pose(*state) for state in Explore_Plan]
		self.cur_goal_idx = 0
		
		self.expl_goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
		
		self.trans_listener = tf.TransformListener()

	def cur_goal(self):
		return self.expl_goals[self.cur_goal_idx]
	
	def next_goal(self):
		""" increments counter to next exploation goal; returns False if none left """

		if self.cur_goal_idx < (len(self.expl_goals) - 1):
			self.cur_goal_idx += 1
			return True
		else:
			return False

	def is_close(self, pose0, pose1):
		x0, y0, theta0 = get_state(pose0)
		x1, y1, theta1 = get_state(pose1)
		return (abs(x1-x0) < POS_EPS and abs(y1-y0) < POS_EPS and abs(theta0-theta1) < THETA_EPS)

	def loop(self):
		try:
			(translation,rotation) = self.trans_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
			self.x = translation[0]
			self.y = translation[1]
			euler = tf.transformations.euler_from_quaternion(rotation)
			self.theta = euler[2]
			self.pose = state_to_pose(self.x, self.y, self.theta)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			pass
		
		# Logs the current mode
        if not(self.last_mode_printed == self.mode):
            rospy.loginfo("Current Mode: %s", self.mode)
            self.last_mode_printed = self.mode
		
		if self.mode == Mode.POSE:
			if self.is_close(self.pose, self.cur_goal()):
				self.mode = Mode.STOP
			else:
				self.expl_goal_publisher(self.cur_goal())
				
		elif self.mode == Mode.STOP:
			rospy.loginfo("Arrived at Exploration Goal %d", self.cur_goal_idx)
			if self.next_goal():
				self.mode = Mode.POSE
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
    sup = Explorer()
    sup.run()
