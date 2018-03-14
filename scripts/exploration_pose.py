import rospy
from geometry_msgs.msg import PoseStamped
from supervisor import POSE_EPS, THETA_EPS
from utils import state_to_pose, pose_to_state
import tf
import math

def is_close(pose0, pose1):
    """ checks if the robot is at a pose within some threshold """
    
	x0, y0, theta0 = pose_to_state(pose0)
	x1, y1, theta1 = pose_to_state(pose1)
    return (abs(x1-x0) < POS_EPS and abs(y1-y0) < POS_EPS and abs(theta0-theta1) < THETA_EPS)

# TODO: Define list of poses for exploring the map.
trans_listener = tf.TransformListener()
expl_goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)


# Defining goal pose locations
current_plan = [[0., 0.,], [0., 5.]]
expl_poses = [];
for state in current_plan:
    pose_st = PoseStamped()
    pose_st.pose.position.x = state[0]
    pose_st.pose.position.y = state[1]
    pose_st.pose.orientation.w = 1
    pose_st.header.frame_id = 'map'
    expl_poses.append(pose_st)


eps = 0.1
rate = rospy.Rate(10)  # 10 Hz
for goal in expl_poses: # list of goal poses
	expl_goal_publisher.publish(goal)
	while not is_close(cur, goal):
		rate.sleep()
		try:
            (translation,rotation) = trans_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
            x = translation[0]
            y = translation[1]
            euler = tf.transformations.euler_from_quaternion(rotation)
            theta = euler[2]
            cur = state_to_pose(x, y, theta)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
