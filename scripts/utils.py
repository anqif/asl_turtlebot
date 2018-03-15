import numpy as np
from geometry_msgs.msg import PoseStamped

def wrapToPi(a):
    if isinstance(a, list):
        return [(x + np.pi) % (2*np.pi) - np.pi for x in a]
    return (a + np.pi) % (2*np.pi) - np.pi

def state_to_pose(x,y,theta = None):
	""" convert state (x,y,theta) to PoseStamped """
	
	p = PoseStamped()
	p.pose.position.x = x
	p.pose.position.y = y
	p.pose.position.z = 0
	
	if theta is None:
		p.pose.orientation.w = 1
	else:
		quaternion = tf.transformations.quaternion_from_euler(0,0,theta)
		p.pose.orientation.x = quaternion[0]
		p.pose.orientation.y = quaternion[1]
		p.pose.orientation.z = quaternion[2]
		p.pose.orientation.w = quaternion[3]
	p.header.frame_id = 'map'   # TODO: Is this necessary?
	return p

def pose_to_state(msg):
	""" convert PoseStamped to state (x,y,theta) """
	
	x = msg.pose.position.x
	y = msg.pose.position.y
	rotation = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
    euler = tf.transformations.euler_from_quaternion(rotation)
    theta = euler[2]
	return x, y, theta
