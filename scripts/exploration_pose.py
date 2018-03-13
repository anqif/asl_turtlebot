import rospy
from geometry_msgs.msg import PoseStamped
from supervisor import POSE_EPS, THETA_EPS
import tf
import math

def get_pose(x,y,theta):
	""" convert state (x,y,theta) to PoseStamped """
	
	p = PoseStamped()
	p.pose.position.x = x
	p.pose.position.y = y
	p.pose.position.z = 0
	
	quaternion = tf.transformations.quaternion_from_euler(0,0,theta)
	p.pose.orientation.x = quaternion[0]
	p.pose.orientation.y = quaternion[1]
	p.pose.orientation.z = quaternion[2]
	p.pose.orientation.w = quaternion[3]
	return p

def get_state(msg):
	""" convert PoseStamped to state (x,y,theta) """
	
	x = msg.pose.position.x
	y = msg.pose.position.y
	rotation = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
    euler = tf.transformations.euler_from_quaternion(rotation)
    theta = euler[2]
	return x, y, theta

def is_close(pose0, pose1):
    """ checks if the robot is at a pose within some threshold """
    
	x0, y0, theta0 = get_state(pose0)
	x1, y1, theta1 = get_state(pose1)
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
            cur = get_pose(x, y, theta)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
