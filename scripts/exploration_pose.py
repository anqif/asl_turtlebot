import rospy
from geometry_msgs.msg import PoseStamped
from supervisor import POSE_EPS, THETA_EPS

def close_to(self,x,y,theta):
    """ checks if the robot is at a pose within some threshold """

    return (abs(x-self.x)<POS_EPS and abs(y-self.y)<POS_EPS and abs(theta-self.theta)<THETA_EPS)

# TODO: Define list of poses for exploring the map.
expl_goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)


# Defining goal pose locations
expl_poses = [];
pose_1 = PoseStamped()
pose_1.pose.position.x = 0.
pose_1.pose.position.y = 0.
pose_1.pose.orientation.w = 1
pose_.header.frame_id = 'map'


rate = rospy.Rate(10)  # 10 Hz
for pose in expl_poses: # list of goal poses
	expl_goal_publisher.publish(pose)
	# TODO: Wait until goal is reached, e.g. by using close_to from supervisor.py
	rate.sleep()