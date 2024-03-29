#!/usr/bin/env python

import numpy as np
import rospy
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import Twist, PoseArray, Pose2D
from asl_turtlebot.msg import DetectedObject
import tf
import math
from enum import Enum

# hardcoded pose goal
# x_g = 1.5
# y_g = -4.0
# theta_g = 0

# threshold at which we consider the robot at a location
POS_EPS = .1
THETA_EPS = .15

# time to stop at a stop sign
STOP_TIME = 1

# minimum distance from a stop sign to obey it
STOP_MIN_DIST = .5

# time taken to cross an intersection
#can change depend on the distance or just tune to choose a safe value
CROSSING_TIME = 1

#detected distance and current distance for determing if rescue the right animal
detected_dist_threshold = 0.1 #m

# self.detected: # of detected animals
# self.all: # of all animals
# self.rescued: # of rescued animals
# self.PreMode: previous mode used for determing which mode to go back after stop/cross

# state machine modes, not all implemented
class Mode(Enum):
    IDLE = 1
    EXPLORE = 2
    STOP = 3
    CROSS = 4
    RESCUE = 5
    BTOG = 6 # back to garage
    COMM = 7
    MANUAL = 8

class Supervisor:
    """ the state machine of the turtlebot """

    def __init__(self):
        rospy.init_node('turtlebot_supervisor', anonymous=True)

        # current pose
        self.x = 0
        self.y = 0
        self.theta = 0

        # current mode
        self.mode = Mode.RESCUE#EXPLORE
        self.last_mode_printed = None

        self.nav_goal_publisher = rospy.Publisher('/cmd_nav', Pose2D, queue_size=10)
        self.pose_goal_publisher = rospy.Publisher('/cmd_pose', Pose2D, queue_size=10)
        # self.rescue_rdy_publisher = rospy.Publisher('/ready_to_rescue', bool, queue_size=10)

        rospy.Subscriber('/detector/stop_sign', DetectedObject, self.stop_sign_detected_callback)
        rospy.Subscriber('/detector/cat', DetectedObject, self.animal_detected_callback)
        rospy.Subscriber('/detector/dog', DetectedObject, self.animal_detected_callback)
        rospy.Subscriber('/cmd_pose', Pose2D, self.goal_pose_callback)
        # rospy.Subscriber('/rescue_on', bool, self.comm_callback)

        #constructor
        self.trans_listener = tf.TransformListener()
        #self.trans_listener2 = tf.TransformListener()

        # # of detected animals
        self.detected = 3
        #all # of animals
        self.all = 5
        # # of rescued animals
        self.rescued = 0
        #PreMode for going back to the previous meaningful mode after stopped
        self.PreMode = self.mode
        #list of detected animals x,y (no th needed)
        self.Detected_animal_list = [[3,3],[4,1],[1,1]]

        #list of edtected stop signs (x,y) (no th needed)
        self.Detected_stop_list = []

        self.pose_garage = np.array([0,0, np.pi])

        self.GarageReached = True


    def animal_detected_callback(self, msg):
        if self.mode == Mode.EXPLORE:
            self.PreMode = Mode.EXPLORE
            self.mode = Mode.STOP

            #compute distance from bounding box around detected object
            ymin,xmin,ymax,xmax = msg.corners
            dx = xmax-xmin
            dy = ymax-ymin
            area = dx*dy
            a = -1.433130624175689*(10**-5)
            b = 0.4485
            box_dist = a * area + b
            print 'name: ', msg.name, 'animal dist: ', box_dist
            x_curr_animal = self.x + box_dist * np.cos(self.theta)
            y_curr_animal = self.y + box_dist * np.sin(self.theta)
            print 'x_an,y_an: ', x_curr_animal, y_curr_animal
            self.animal_num += 1
            self.Detected_animal_list.append((x_curr_animal, y_curr_animal)) # add animal to list

        #need to change detected_pos thing, idea is to make sure the newly rescued animal is among the previously detected animal
        if (self.mode == Mode.RESCUE): #and self.close_to_wo_th(self.Detected_animal_list[self.rescued][0], self.Detected_animal_list[self.rescued][1]):
            self.PreMode = Mode.RESCUE
            self.mode = Mode.STOP
            self.rescued += 1

            ymin,xmin,ymax,xmax = msg.corners
            dx = xmax-xmin
            dy = ymax-ymin
            area = dx*dy
            a = -1.433130624175689*(10**-5)
            b = 0.4485
            box_dist = a * area + b
            print 'name: ', msg.name, 'box dist: ', box_dist
            self.x_curr_stop = self.x + box_dist * np.cos(self.theta)
            self.y_curr_stop = self.y + box_dist * np.sin(self.theta)
            self.Detected_stop_list += (self.x_curr_stop, self.y_curr_stop)
            print 'x_stop, y_stop: ', self.x_curr_stop, self.y_curr_stop

    def stop_sign_detected_callback(self, msg):
        """ callback for when the detector has found a stop sign. Note that
        a distance of 0 can mean that the lidar did not pickup the stop sign at all
        so you shouldn't necessarily stop then """

        if (self.mode == Mode.EXPLORE) or (self.mode == Mode.RESCUE) or (self.mode == Mode.BTOG):
            self.PreMode = self.mode
            self.mode = Mode.STOP

    def comm_callback(self,msg):
        #assume msg is a boolean indicating that we can start rescuing
        if msg:
            self.mode = Mode.RESCUE

    def goal_pose_callback(self, msg):
        x, y, th = msg
        self.x_g = x
        self.y_g = y
        self.th_g = th

    def set_goal_pose(self):
        if self.mode == Mode.RESCUE:
            self.x_g = self.Detected_animal_list[self.rescued-1][0]
            self.y_g = self.Detected_animal_list[self.rescued-1][1]
            self.th_g = 0
        elif self.mode == Mode.BTOG:
            self.x_g = self.pose_garage[0]
            self.y_g = self.pose_garage[1]
            self.th_g = 0

    def nav_to_pose(self):
        """ sends the current desired pose to the naviagtor """
        nav_g_msg = Pose2D()

        nav_g_msg.x = self.x_g
        nav_g_msg.y = self.y_g
        nav_g_msg.theta = self.th_g

        self.nav_goal_publisher.publish(nav_g_msg)

    def close_to(self,x,y,theta):
        """ checks if the robot is at a pose within some threshold """
        return (abs(x-self.x)<POS_EPS and abs(y-self.y)<POS_EPS and abs(theta-self.theta)<THETA_EPS)

    def close_to_wo_th(self,x,y):
        """ check if the position of the robot is close enough to the animal being rescued """
        print type(x)
        print type(self.x)
        return (abs(x-self.x)<POS_EPS and abs(y-self.y)<POS_EPS)

    # def go_to_pose(self):
    #     """ sends the current desired pose to the pose controller """

    #     pose_g_msg = Pose2D()
    #     pose_g_msg.x = x_g
    #     pose_g_msg.y = y_g
    #     pose_g_msg.theta = theta_g

    #     self.pose_goal_publisher.publish(pose_g_msg)

    def loop(self):

        """ the main loop of the robot. At each iteration, depending on its
        mode (i.e. the finite state machine's state), if takes appropriate
        actions. This function shouldn't return anything """

        #listen to where the robot is in ground frame
        try:
            (trans,rot) = self.trans_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
            self.x = trans[0]
            self.y = trans[1]
            euler = tf.transformations.euler_from_quaternion(rot)
            self.theta = euler[2]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        # print 'x: ', self.x
        # print 'y: ', self.y
        # print 'th:', self.theta

        #listen to frame transformation from camera to Velodyne
        # try:
        #     #print 'update x y th! in loop'
        #     (trans2,rot2) = self.trans_listener2.lookupTransform('/camera', '/Velodyne', rospy.Time(0))
        #     trans_x = trans[0]
        #     trans_y = trans[1]
        #     euler = tf.transformations.euler_from_quaternion(rot)
        #     trans_theta = euler[2]
        # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #     pass
        # print 'trans_x: ', trans_x
        # print 'trans_y: ', trans_y
        # print 'euler angle?: ', euler
        #need form transformation matrix?

        # logs the current mode
        if not(self.last_mode_printed == self.mode):
            rospy.loginfo("Current Mode: %s", self.mode)
            self.last_mode_printed = self.mode

        # checks wich mode it is in and acts accordingly
        if self.mode == Mode.IDLE:
            # not doing anything
            rospy.loginfo("Idling..")

        elif self.mode == Mode.EXPLORE:
            # animal or stop sign detected
            if self.detected == self.all:
                self.PreMode = Mode.EXPLORE
                self.mode = Mode.BTOG
                #TODO publish the animal list to rescue
            else:
                #explore
                self.PreMode = Mode.EXPLORE
                rospy.loginfo("exploring...")

        elif self.mode == Mode.STOP:
            # cannot use sleep as we would stil like to record/calculate animal positions
            rospy.loginfo("Stopped...")
            curr_time_sec = rospy.get_time()
            print curr_time_sec
            while (rospy.get_time() - curr_time_sec <= STOP_TIME):
                pass
            print 'time after stop: ', rospy.get_time()
            self.mode = Mode.CROSS

        elif self.mode == Mode.CROSS:
            rospy.loginfo("crossing...")
            curr_time_sec2 = rospy.get_time()
            print curr_time_sec2
            if (self.PreMode == Mode.EXPLORE) or (self.PreMode == Mode.RESCUE) or (self.PreMode == Mode.BTOG):
                while (rospy.get_time() - curr_time_sec2 <= CROSSING_TIME):
                    self.nav_to_pose()
                print 'time after cross: ', rospy.get_time()
                self.mode = self.PreMode

        elif self.mode == Mode.RESCUE:
            self.PreMode = self.mode
            self.set_goal_pose()
            if self.rescued != self.detected:
                self.nav_to_pose()
            else:
                self.mode = Mode.BTOG

        elif self.mode == Mode.COMM:
            # subscribe to receive signal of start rescuing
            rospy.loginfo("wait for rescue...")
            self.rescue_rdy_publisher.publish(True)

        elif self.mode == Mode.BTOG:
            self.set_goal_pose()
            self.PreMode = self.mode
            self.GarageReached = self.close_to_wo_th(self.pose_garage[0], self.pose_garage[1])
            if (self.GarageReached) and (self.rescued == self.detected):
                self.mode = Mode.IDLE
            elif self.GarageReached == 0:
                self.nav_to_pose()

        else:
            raise Exception('This mode is not supported: %s'
                % str(self.mode))

    def run(self):
        rate = rospy.Rate(5) # 5 Hz
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()

if __name__ == '__main__':
    sup = Supervisor()
    sup.run()
