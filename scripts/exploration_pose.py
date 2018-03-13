import rospy
from geometry_msgs.msg import PoseStamped
from supervisor import POSE_EPS, THETA_EPS

def close_to(self,x,y,theta):
    """ checks if the robot is at a pose within some threshold """

    return (abs(x-self.x)<POS_EPS and abs(y-self.y)<POS_EPS and abs(theta-self.theta)<THETA_EPS)

# TODO: Define list of poses for exploring the map.
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
for pose in expl_poses: # list of goal poses
	expl_goal_publisher.publish(pose)
	# TODO: Wait until goal is reached, e.g. by using close_to from supervisor.py
	
	rate.sleep()