#!/usr/bin/env python

import rospy
import cv2
import cv_bridge
from smach import State, StateMachine
import smach_ros
from dynamic_reconfigure.server import Server
from comp5.cfg import Comp5Config
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Twist, Pose, PoseStamped, PointStamped
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import BumperEvent, Sound, Led
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import decompose_matrix, compose_matrix, quaternion_from_euler
from ros_numpy import numpify
import actionlib
from sensor_msgs.msg import Joy, LaserScan, Image
import numpy as np
import angles as angles_lib
import math
import random
from std_msgs.msg import Bool, String, Int32
import imutils
from copy import deepcopy
import tf

START = False    
FORWARD_CURRENT = 0
TURN_CURRENT = 0
POSE = [0, 0, 0]
turn_direction = 1
PHASE = None
SHAPE = "triangle"
SHAPE_MATCHED = False
NUM_SHAPES = 0
CURRENT_CHECKPOINT = 0
UNKNOWN_CHECKPOINT = 0
PHASE4_TASK_COMPLETED = 0

BRIDGE = cv_bridge.CvBridge()
IMAGE = None

TB_POSE = None

GREEN_FOUND = False

PHASE4_SHAPE_FOUND = False
BOX_ID = 1
PHASE3_LINE_FOUND = False
PHASE4_BOX_FOUND = False
PHASE4_GOAL_FOUND = False
PHASE4_BOX_CHECKPOINT = ""
PHASE4_GOAL_CHECKPOINT = ""
PHASE4_BOX_X = 0
PHASE4_GOAL_X = 0
PHASE4_FACING = "" # either box or goal
PHASE4_EXIT_GOAL = None
PHASE4_DELTA_X = 0
PHASE4_PUSH_Y = 0
PHASE4_START_ADJUST= False
PHASE4_FINISHED_ADJUST = False
class WaitForButton(State):
    def __init__(self):
        State.__init__(self, outcomes=["pressed", "exit"])
        self.rate = rospy.Rate(10)

    def execute(self, userdata):
        global START
        while not START and not rospy.is_shutdown():
            self.rate.sleep()
        if rospy.is_shutdown():
            return "exit"
        else:
            return "pressed"


class FollowLine(State):
    """
    should follow the white line, until see full red
    """

    def __init__(self, phase="1.0"):
        State.__init__(self, outcomes=[
                       "see_red", "exit", "failure", "see_nothing", "see_long_red", "all_done"])
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.phase = phase
        self.green_start_pub = rospy.Publisher(
            'green_start', Bool, queue_size=1)
        self.find_green = False
        self.red_object_count = 0
        self.reset()

    def reset(self):
        self.twist = Twist()
        self.found_object = False
        self.start_timeout = False
        self.temporary_stop = False
        self.image_received = False
        self.white_line_ended = False
        self.image = None
        self.object_area = 0
        self.dt = 1.0 / 20.0

    def execute(self, userdata):
        global PHASE, FORWARD_CURRENT, TURN_CURRENT, linear_vel, red_timeout, Kp, Kd, Ki, IMAGE, GREEN_FOUND, PHASE3_LINE_FOUND

        FORWARD_CURRENT = 0.0
        TURN_CURRENT = 0.0
        previous_error = 0
        integral = 0
        sleep_duration = rospy.Duration(self.dt, 0)
        PHASE = self.phase

        self.reset()
        start_time = None
        self.image = None
        # self.red_object_count = 0

        while not rospy.is_shutdown() and START:
            self.image = IMAGE
            self.find_green = GREEN_FOUND

            if self.image is None:
                continue

            image = deepcopy(self.image)
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV_FULL)

            lower_white = np.array([white_min_h, white_min_s, white_min_v])
            upper_white = np.array([white_max_h, white_max_s, white_max_v])

            mask = cv2.inRange(hsv, lower_white, upper_white)

            hsv2 = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            lower_red = np.array([red_min_h,  red_min_s,  red_min_v])
            upper_red = np.array([red_max_h, red_max_s, red_max_v])

            mask_red = cv2.inRange(hsv2, lower_red, upper_red)

            # lower_green = np.array([108, 68, 100])
            # upper_green = np.array([200, 255, 255])
            # mask_green = cv2.inRange(hsv, lower_green, upper_green)

            cx = cy = w = None
            h, w, d = image.shape
            search_top = 3*h/4
            search_bot = 3*h/4 + 20

            mask[0:search_top, 0:w] = 0
            mask[search_bot:h, 0:w] = 0
            M = cv2.moments(mask)

            if M['m00'] > 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)
            
            if self.phase == "3.1" and M['m00'] <= 0 and not PHASE3_LINE_FOUND:
                Turn(180).execute(None)

                while M['m00'] <= 0:
                    self.twist.linear.x = 0
                    self.twist.angular.z = 0.55
                    self.cmd_vel_pub.publish(self.twist)
                
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)
            elif self.phase == "3.1" and M['m00'] > 0:
                PHASE3_LINE_FOUND = True


            if self.phase == "4.2" and M['m00'] == 0:  # no more white line ahead
                self.start_timeout = True
            elif self.phase == "2.1":
                if self.find_green and M['m00'] == 0:
                    self.start_timeout = True
            elif self.phase != "4.2":
                max_area = 0

                if self.phase == "3.1":
                    mask_red[0:h/2, 0:w] = 0

                _, contours, _ = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                # total_area = sum([cv2.contourArea(x) for x in contours])
                contours = [x for x in contours if cv2.contourArea(x) > 10]

                if len(contours) > 0:  # Keep the maximum sum of area of red objects since first detection
                    self.found_object = True
                    max_area = max([cv2.contourArea(x) for x in contours])
                    self.object_area = max(self.object_area, max_area)

                if len(contours) == 0 and self.found_object:  # red objects no more visible since last detection
                    self.found_object = False
                    thresh = 1000
                    if self.phase=="2.2":
                        thresh = 1000
                        print(".....!!!!", self.object_area)
                    
                    if self.object_area > thresh:  # valid small red object in front
                        self.red_object_count += 1

                        if self.phase == "4.1":
                            self.temporary_stop = True  # temporary stop regardless of the length of red object in phase 4.1 (because the long red object sometimes get misclassified as a small red one because of the turn)
                        elif self.phase == "1.0":
                            if self.red_object_count == 1:
                                self.start_timeout = True
                        elif self.phase == "2.0":
                            if self.red_object_count == 1:
                                self.temporary_stop = True
                            elif self.red_object_count == 2:
                                self.start_timeout = True
                        elif self.phase == "2.2":
                            if self.red_object_count == 1:
                                self.start_timeout = True
                        elif self.phase == "4.1":
                            if self.red_object_count == 1:
                                self.temporary_stop = True
                        elif self.phase == "3.1":
                            if self.red_object_count == 1:
                                self.temporary_stop = True
                            elif self.red_object_count > 4:
                                return "all_done"
                            elif self.red_object_count >= 2:
                                self.start_timeout = True
                                
                    
                    self.object_area = 0
                    
            cv2.imshow("window", mask_red)
            cv2.waitKey(3)

            if self.phase == "2.1":
                self.green_start_pub.publish(Bool(True))

            speed = linear_vel
            local_kp = Kp
            local_kd = Kd
            local_ki = Ki

            if self.phase == "2.1" or self.phase == "2.2":
                speed = 0.2
                local_kp = 1.0 / 300.0
                local_kd = 1.0 / 700.0
            # else:
            #     Kp = 1.0 / 400.0
            #     Kd = 1.0 / 700.0


            if self.temporary_stop:
                rospy.sleep(rospy.Duration(2.0))
                self.temporary_stop = False
                
                if self.phase == "4.1":
                    self.start_timeout = True

            if self.start_timeout and start_time is None:
                start_time = rospy.Time.now()

            r_timeout = red_timeout
            if self.phase=="2.2" or self.phase=="2.1":
                r_timeout = rospy.Duration(0)
            if self.start_timeout and start_time + r_timeout < rospy.Time.now():
                start_time = None
                self.start_timeout = False

                if self.phase == "4.1":
                    return "see_long_red"
                elif self.phase == "4.2":
                    return "see_nothing"
                elif self.phase == "2.1":
                    self.green_start_pub.publish(Bool(False))
                    return "see_red"
                else:
                    return "see_red"

            # BEGIN CONTROL
            if cx is not None and w is not None:
                error = float(cx - w/2.0)
                integral += error * self.dt
                derivative = (error - previous_error) / self.dt

                self.twist.linear.x = speed
                self.twist.angular.z = -(local_kp * float(error) + local_kd * derivative + local_ki * integral)
                self.cmd_vel_pub.publish(self.twist)

                previous_error = error

                rospy.sleep(sleep_duration)
            # END CONTROL
        
        if not START:
            return "exit"


def calc_delta_vector(start_heading, distance):
    dx = distance * np.cos(start_heading)
    dy = distance * np.sin(start_heading)
    return np.array([dx, dy])


def check_forward_distance(forward_vec, start_pos, current_pos):
    current = current_pos - start_pos
    # vector projection (project current onto forward_vec)
    delta = np.dot(current, forward_vec) / \
        np.dot(forward_vec, forward_vec) * forward_vec
    dist = np.sqrt(delta.dot(delta))
    return dist


class MoveBaseGo(State):
    def __init__(self, distance = 0, yaw = 0):
        State.__init__(self, outcomes=["success", "exit", 'failure'])
        self.distance = distance
        self.yaw = yaw
        self.move_base_client = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction)

    def execute(self, userdata):
        if START and not rospy.is_shutdown():

            quaternion = quaternion_from_euler(0, 0, self.yaw)

            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "base_footprint"
            goal.target_pose.pose.position.x = self.distance
            goal.target_pose.pose.position.y = 0
            goal.target_pose.pose.orientation.x = quaternion[0]
            goal.target_pose.pose.orientation.y = quaternion[1]
            goal.target_pose.pose.orientation.z = quaternion[2]
            goal.target_pose.pose.orientation.w = quaternion[3]
            
            self.move_base_client.send_goal_and_wait(goal)

            return "success"


class Translate(State):
    def __init__(self, distance=0.15, linear=-0.2):
        State.__init__(self, outcomes=["success", "exit", 'failure'])
        self.tb_position = None
        self.tb_rot = [0,0,0,0]
        self.distance = distance
        self.COLLISION = False
        self.linear = linear

        # pub / sub
        self.cmd_pub = rospy.Publisher(
            "cmd_vel", Twist, queue_size=1)

    def update_using_odom(self):
        global TB_POSE

        __, __, angles, position, __ = decompose_matrix(numpify(TB_POSE))
        self.tb_position = position[0:2]
        self.tb_rot = angles

    def execute(self, userdata):
        global turn_direction, TB_POSE
        global START
        if not START:
            return 'quit'

        while TB_POSE is None:
            continue
        
        self.update_using_odom()

        self.COLLISION = False
        start_heading = self.tb_rot[2]
        start_pos = self.tb_position
        forward_vec = calc_delta_vector(start_heading, self.distance)
        rate = rospy.Rate(30)

        while not rospy.is_shutdown():
            self.update_using_odom()

            dist = check_forward_distance(
                forward_vec, start_pos, self.tb_position)
            if dist > self.distance:
                return "success"
            if self.linear > 0 and self.COLLISION:
                msg = Twist()
                msg.linear.x = 0.0
                self.cmd_pub.publish(msg)
                self.COLLISION = False
                # hit something while driving perpendicular so swap turn direction
                turn_direction = -1 * turn_direction
                return "collision"

            msg = Twist()
            msg.linear.x = self.linear
            self.cmd_pub.publish(msg)
            rate.sleep()


class Turn(State):
    """
    Turning a specific angle, based on Sean's example code from demo2
    """

    def __init__(self, angle=90, radian=False):
        State.__init__(self, outcomes=["success", "exit", 'failure'])
        self.tb_position = None
        self.tb_rot = None
        # angle defines angle target relative to goal direction
        self.angle = angle
        self.radian = radian
        # pub / sub
        self.cmd_pub = rospy.Publisher(
            "cmd_vel", Twist, queue_size=1)

    def update_using_odom(self):
        global TB_POSE
        
        __, __, angles, position, __ = decompose_matrix(numpify(TB_POSE))
        self.tb_position = position[0:2]
        self.tb_rot = angles

    def execute(self, userdata):
        global turn_direction, TB_POSE
        global START
        global POSE

        if not START:
            return 'exit'

        while TB_POSE is None:
            continue
        
        self.update_using_odom()

        start_pose = POSE
        if self.angle == 0:  # target is goal + 0
            goal = start_pose[2]
        elif self.angle == 90:  # target is goal + turn_direction * 90
            goal = start_pose[2] + np.pi/2 * turn_direction
        elif self.angle == 180:  # target is goal + turn_direction * 180
            goal = start_pose[2] + np.pi * turn_direction
        elif self.angle == -90:  # target is goal + turn_direction * 270
            goal = start_pose[2] - np.pi/2 * turn_direction
        elif self.angle == -100:  # target is goal + turn_direction * 270
            goal = start_pose[2] - 5*np.pi/9 * turn_direction
        elif self.angle == 120:
            goal = start_pose[2] + 2*np.pi/3 *turn_direction
        elif self.angle == 135:
            goal = start_pose[2] + 150*np.pi/180 * turn_direction
        else:
            goal = start_pose[2] + self.angle*np.pi/180 * turn_direction

        if self.radian:
            goal = start_pose[2] + self.angle * turn_direction

        goal = angles_lib.normalize_angle(goal)

        cur = np.abs(angles_lib.normalize_angle(self.tb_rot[2]) - goal)
        speed = 0.75
        rate = rospy.Rate(30)

        direction = turn_direction

        # if 2 * np.pi - angles_lib.normalize_angle_positive(goal) < angles_lib.normalize_angle_positive(goal) or self.angle == 0:
        if angles_lib.normalize_angle(angles_lib.normalize_angle(self.tb_rot[2]) - goal) > 0:
            direction = turn_direction * -1

        while not rospy.is_shutdown():
            self.update_using_odom()

            cur = np.abs(angles_lib.normalize_angle(self.tb_rot[2]) - goal)

            # slow down turning as we get closer to the target heading
            if cur < 0.1:
                speed = 0.15
            if cur < 0.0571:
                break
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = direction * speed
            self.cmd_pub.publish(msg)
            rate.sleep()

        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_pub.publish(msg)

        return 'success'


class DepthCount(State):
    """
    Count the number of objects based on the depth image
    """

    def __init__(self):
        State.__init__(self, outcomes=["success", "exit", "failure"],
                       output_keys=["object_count"])

        self.bridge = cv_bridge.CvBridge()

        self.count_start = False
        self.object_count = 0
        self.count_finished = False
        self.count1_start_pub = rospy.Publisher('start1', Bool, queue_size=1)

    def execute(self, userdata):
        global START

        self.count_start == False
        self.object_count = 0
        self.count_finished = False

        self.count1_start_pub.publish(Bool(True))
        count1_sub = rospy.Subscriber('count1', Int32, self.count_callback)

        while not rospy.is_shutdown() and START and not self.count_finished:
            pass

        if self.object_count > 3:
            self.object_count = 3

        userdata.object_count = self.object_count
        return "success"

        if not START:
            return "exit"

    def count_callback(self, msg):
        self.object_count = msg.data
        self.count1_start_pub.publish(Bool(False))
        self.count_finished = True


class Signal1(State):
    """
    Make sound and LED lights based on count for phase 1
    """

    def __init__(self):
        State.__init__(self, outcomes=["success", "exit", "failure"],
                       input_keys=["object_count"])
        self.led1_pub = rospy.Publisher(
            "/mobile_base/commands/led1", Led, queue_size=1)
        self.led2_pub = rospy.Publisher(
            "/mobile_base/commands/led2", Led, queue_size=1)
        self.sound_pub = rospy.Publisher(
            '/mobile_base/commands/sound', Sound, queue_size=1)

    def execute(self, userdata):
        global START

        print userdata.object_count

        if START and not rospy.is_shutdown():
            if userdata.object_count == 1:
                self.led1_pub.publish(Led(1))
            elif userdata.object_count == 2:
                self.led2_pub.publish(Led(1))
            elif userdata.object_count == 3:
                self.led1_pub.publish(Led(1))
                self.led2_pub.publish(Led(1))
        for _ in range(userdata.object_count):
            self.sound_pub.publish(Sound(0))
            rospy.sleep(rospy.Duration(0.5))
        return "success"
        if not START:
            return "exit"


class LookForSide(State):
    """
    Looking for the half red line while following the white line
    """

    def __init__(self):
        State.__init__(self, outcomes=["see_half_red", "exit", "failure"])

    def execute(self, userdata):
        global START

        if not START:
            return "exit"


class FindGreen(State):
    """
    Recognize the Green image shape based on NN?
    """

    def __init__(self):
        State.__init__(self, outcomes=["success", "exit", "failure"])
        self.shape2_sub = rospy.Subscriber(
            "/shape2", String, self.shapeCallback)
        self.count2_sub = rospy.Subscriber(
            "/count2", Int32, self.countCallback)
        self.done_shape = False
        self.done_count = False

        self.count2_pub = rospy.Publisher("start2", Bool, queue_size=1)
        self.led1_pub = rospy.Publisher(
            "/mobile_base/commands/led1", Led, queue_size=1)
        self.led2_pub = rospy.Publisher(
            "/mobile_base/commands/led2", Led, queue_size=1)

        self.sound_pub = rospy.Publisher(
            "/mobile_base/commands/sound", Sound, queue_size=1)

    def shapeCallback(self, msg):
        global SHAPE
        print("GOT SHAPE", msg)
        SHAPE = msg.data
        self.done_shape = True

    def countCallback(self, msg):
        print("GOT COUNT", msg)
        if msg.data == 1:
            self.led1_pub.publish(Led(1))
        elif msg.data == 2:
            self.led2_pub.publish(Led(1))
        elif msg.data == 3:
            self.led1_pub.publish(Led(1))
            self.led2_pub.publish(Led(1))

        for _ in range(msg.data):
            self.sound_pub.publish(Sound(0))
            rospy.sleep(rospy.Duration(0.5))

        self.done_count = True

    def execute(self, userdata):
        global START, SHAPE

        self.led1_pub.publish(Led(0))
        self.led2_pub.publish(Led(0))

        self.count2_pub.publish(Bool(data=True))
        print("published")

        while not rospy.is_shutdown():
            if self.done_shape and self.done_count:
                break
            rospy.Rate(10).sleep()

        if not START:
            return "exit"
        return "success"


class MoveForward(State):
    """
    Move forward a bit until can't see the red
    """

    def __init__(self):
        State.__init__(self, outcomes=["no_more_red", "exit", "failure"])

    def execute(self, userdata):
        global START

        if not START:
            return "exit"


class CheckShape(State):
    """
    determine if the RED pattern is the shape we see in phase 3
    """

    def __init__(self):
        State.__init__(self, outcomes=["matched", "exit", "failure"])
        self.start3_pub = rospy.Publisher("start3", Bool, queue_size=1)
        self.shape3_sub = rospy.Subscriber(
            "shape3", String, self.shapeCallback)

        self.shape = None
        self.got_shape = False

    def shapeCallback(self, msg):

        self.got_shape = True
        print("GOT SHAPE=", msg.data)
        self.shape = msg.data

    def execute(self, userdata):
        global START, SHAPE, PHASE4_SHAPE_FOUND

        self.start3_pub.publish(Bool(True))
        self.got_shape = False
        while not rospy.is_shutdown():
            if self.got_shape:
                break
            rospy.Rate(10).sleep()

        if self.shape == SHAPE:
            PHASE4_SHAPE_FOUND = True
            return "matched"
        elif NUM_SHAPES >= 3:
            return "matched"

        else:
            print("failed?", self.shape)
            return "failure"
        if not START:
            return "exit"

class CheckShape2(State):
    """
    determine if the RED pattern is the shape we see in phase 3
    """

    def __init__(self):
        State.__init__(self, outcomes=["matched", "exit", "failure"])
        self.start3_pub = rospy.Publisher("start5", Bool, queue_size=1)
        self.shape3_sub = rospy.Subscriber(
            "shape5", String, self.shapeCallback)

        self.shape = None
        self.got_shape = False

    def shapeCallback(self, msg):

        self.got_shape = True
        print("GOT SHAPE=", msg.data)
        self.shape = msg.data

    def execute(self, userdata):
        global START, SHAPE_MATCHED, SHAPE

        self.start3_pub.publish(Bool(True))
        self.got_shape = False
        while not rospy.is_shutdown():
            if self.got_shape:
                break
            rospy.Rate(10).sleep()

        if self.shape == SHAPE:
            SHAPE_MATCHED = True
            return "matched"
        elif NUM_SHAPES >= 3:
            return "matched"

        else:
            print("failed?", self.shape)
            return "failure"
        if not START:
            return "exit"

class Signal3(State):
    """
    Make a sound for phase 3
    """

    def __init__(self):
        State.__init__(self, outcomes=["success", "exit", "failure"])
        self.sound_pub = rospy.Publisher(
            "mobile_base/commands/sound", Sound, queue_size=1)

    def execute(self, userdata):
        global START, SHAPE_MATCHED
        if SHAPE_MATCHED:
            self.sound_pub.publish(Sound(0))
        return "success"

        if not START:
            return "exit"


def joy_callback(msg):
    global START, UNKNOWN_CHECKPOINT

    if msg.buttons[4] == 1: # LB
        if msg.buttons[0]:
            UNKNOWN_CHECKPOINT = 0
        elif msg.buttons[1]:
            UNKNOWN_CHECKPOINT = 1
        elif msg.buttons[2]:
            UNKNOWN_CHECKPOINT = 2
        elif msg.buttons[3]:
            UNKNOWN_CHECKPOINT = 3


    elif msg.buttons[5] == 1: #RB
        if msg.buttons[0]:
            UNKNOWN_CHECKPOINT = 4
        elif msg.buttons[1]:
            UNKNOWN_CHECKPOINT = 5
        elif msg.buttons[2]:
            UNKNOWN_CHECKPOINT = 6
        elif msg.buttons[3]:
            UNKNOWN_CHECKPOINT = 7

    elif msg.buttons[0] == 1:  # button A
        START = True
    elif msg.buttons[1] == 1:  # button B
        START = False


def dr_callback(config, level):
    global Kp, Kd, Ki, red_area_threshold, red_timeout, linear_vel, white_max_h, white_max_s, white_max_v, white_min_h, white_min_s, white_min_v, red_max_h, red_max_s, red_max_v, red_min_h, red_min_s, red_min_v

    # Kp = config["Kp"]
    # Kd = config["Kd"]
    # Ki = config["Ki"]
    # linear_vel = config["linear_vel"]

    # white_max_h = config["white_max_h"]
    # white_max_s = config["white_max_s"]
    # white_max_v = config["white_max_v"]

    # white_min_h = config["white_min_h"]
    # white_min_s = config["white_min_s"]
    # white_min_v = config["white_min_v"]

    red_max_h = config["red_max_h"]
    red_max_s = config["red_max_s"]
    red_max_v = config["red_max_v"]

    red_min_h = config["red_min_h"]
    red_min_s = config["red_min_s"]
    red_min_v = config["red_min_v"]

    # red_area_threshold = config["red_area_threshold"]
    # red_timeout = rospy.Duration(config["red_timeout"])

    return config


def move(forward_target, turn_target, pub, ramp_rate=0.5):
    """
    modified version of move(forward, turn) from https://github.com/erichsueh/LifePoints-412-Comp1/blob/2e9fc4701c3cdc8e4ab8b04ca1da8581cfdf0c5b/robber_bot.py#L25
    """
    global FORWARD_CURRENT
    global TURN_CURRENT

    twist = Twist()
    new_forward = ramped_vel(FORWARD_CURRENT, forward_target, ramp_rate)
    new_turn = ramped_vel(TURN_CURRENT, turn_target, ramp_rate)
    twist.linear.x = new_forward
    twist.angular.z = new_turn
    pub.publish(twist)

    FORWARD_CURRENT = new_forward
    TURN_CURRENT = new_turn


def ramped_vel(v_prev, v_target, ramp_rate):
    """
    get the ramped velocity
    from rom https://github.com/MandyMeindersma/Robotics/blob/master/Competitions/Comp1/Evasion.py
    """
    if abs(v_prev) > abs(v_target):
        ramp_rate *= 2
    step = ramp_rate * 0.1
    sign = 1.0 if (v_target > v_prev) else -1.0
    error = math.fabs(v_target - v_prev)
    if error < step:  # we can get there in this time so we are done
        return v_target
    else:
        return v_prev + sign*step


class StartParking(State):
    def __init__(self):
        State.__init__(self, outcomes=["ready"])

    def execute(self, userdata):
        pass

def transformPointFromMarker(frame, target="odom", x=0, y=0,z=0):
    listener = tf.TransformListener()
    marker_point = PointStamped()
    marker_point.header.frame_id = frame
    marker_point.header.stamp = rospy.Time(0)
    marker_point.point.x = x
    marker_point.point.y = x
    marker_point.point.z = x

    listener.waitForTransform(
        target, marker_point.header.frame_id, rospy.Time(0), rospy.Duration(4))
    marker_point_transformed = listener.transformPoint(
        target, marker_point)

    return marker_point_transformed

class ParkNext(State):
    def __init__(self, checkpoint):
        State.__init__(self, outcomes=[
                       "see_shape", "see_AR_goal", "see_AR_box", "find_nothing"])

        self.checkpoint_list = [MoveBaseGoal(), MoveBaseGoal(), MoveBaseGoal(
        ), MoveBaseGoal(), MoveBaseGoal(), MoveBaseGoal(), MoveBaseGoal(), MoveBaseGoal()]
        self.move_base_client = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction)
        self.shape_start_pub = rospy.Publisher(
            'startShape4', Bool, queue_size=1)

        self.checkpoint = checkpoint
        self.marker = None
        self.debugSound = Signal4(False,0)
    def reset(self):
        self.marker_data_received = False
        self.found_marker = False
        self.count = 0

    def marker_callback(self, msg):
        global BOX_ID
        if not self.marker_data_received:
            self.marker_data_received = True
        self.count += 1
        if len(msg.markers) > 0 and msg.markers[0].id > 0 and msg.markers[0].id < 9 and not (self.checkpoint in ['point6','point7','point8','exit']):
            # print("found a marker????????????????", msg.markers[0].id)
            if self.checkpoint == "look_for_box":
                # save the box id
                BOX_ID = msg.markers[0].id
                self.debugSound.execute(None)
            self.marker = msg.markers[0]
            self.found_marker = True

    def shape_callback(self, msg):
        self.found_shape = msg.data

    def execute(self, userdata):
        global START, CURRENT_CHECKPOINT, UNKNOWN_CHECKPOINT, PHASE4_TASK_COMPLETED, BOX_ID, TB_POSE
        global PHASE4_BOX_FOUND, PHASE4_GOAL_FOUND, PHASE4_BOX_CHECKPOINT, PHASE4_GOAL_CHECKPOINT, PHASE4_EXIT_GOAL
        global PHASE4_BOX_X, PHASE4_GOAL_X, PHASE4_FACING ,PHASE4_PUSH_Y, PHASE4_DELTA_X

        self.reset()
        marker_sub = rospy.Subscriber(
            'ar_pose_marker_base', AlvarMarkers, self.marker_callback)
        shape_sub = rospy.Subscriber('hasShape4', Bool, self.shape_callback)

        self.shape_start_pub.publish(Bool(True))

        if not rospy.is_shutdown() and START:
            while not self.marker_data_received:
                continue

            self.found_marker = False

            while self.count < 2:
                # print self.count
                rospy.Rate(30).sleep()
                
            # save point 1 as the exit point
            if self.checkpoint == "point1":
                quaternion = quaternion_from_euler(0, 0, -math.pi/2)
                PHASE4_EXIT_GOAL = MoveBaseGoal()
                PHASE4_EXIT_GOAL.target_pose.header.frame_id = "odom"
                PHASE4_EXIT_GOAL.target_pose.pose.position.x = TB_POSE.position.x
                PHASE4_EXIT_GOAL.target_pose.pose.position.y = TB_POSE.position.y
                PHASE4_EXIT_GOAL.target_pose.pose.orientation.x = quaternion[0]
                PHASE4_EXIT_GOAL.target_pose.pose.orientation.y = quaternion[1]
                PHASE4_EXIT_GOAL.target_pose.pose.orientation.z = quaternion[2]
                PHASE4_EXIT_GOAL.target_pose.pose.orientation.w = quaternion[3]

            if self.found_marker and self.checkpoint != "look_for_box":
                # transform the marker pose
                # pose_transformed = transformPointFromMarker("ar_marker_"+str(self.marker.id))
                pose_transformed = self.marker.pose.pose
                marker_sub.unregister()
                
                self.shape_start_pub.publish(Bool(False))

                if abs(TB_POSE.position.x - pose_transformed.position.x) < 0.3: 
                    # found an AR tag at current checkpoint
                    if BOX_ID == self.marker.id:
                        # found the box
                        PHASE4_BOX_FOUND = True
                        PHASE4_BOX_CHECKPOINT = self.checkpoint
                        PHASE4_BOX_X = pose_transformed.position.x
                        PHASE4_PUSH_Y = abs(pose_transformed.position.y - TB_POSE.position.y)
                        PHASE4_DELTA_X = abs(pose_transformed.position.x - TB_POSE.position.x)
                        self.update_initial_pose(pose_transformed)
                        PHASE4_FACING = "box"
                        return "see_AR_box"
                    else:
                        # found the goal
                        PHASE4_GOAL_FOUND = True
                        PHASE4_GOAL_CHECKPOINT = self.checkpoint
                        PHASE4_GOAL_X = pose_transformed.position.x
                        PHASE4_FACING = "goal"
                        return "see_AR_goal"
                else:
                    print("not aligned", abs(TB_POSE.position.x - pose_transformed.position.x))
                    return "find_nothing"
            elif (not PHASE4_SHAPE_FOUND) and self.found_shape:
                self.shape_start_pub.publish(Bool(False))
                return "see_shape"
            else:
                print("Acctually nothing")
                CURRENT_CHECKPOINT += 1
                marker_sub.unregister()
                self.shape_start_pub.publish(Bool(False))
                return "find_nothing"

        marker_sub.unregister()
    
    def update_initial_pose(self, tb_pose):
        global POSE
        Pose()
        __,__, angles, position, __ = decompose_matrix(numpify(tb_pose))
        POSE[2] =angles_lib.normalize_angle( angles[2] )

class Signal4(State):
    def __init__(self, led1, led1color, led2=False, led2color=None, playsound=True):
        State.__init__(self, outcomes=["done"])
        self.led1 = led1
        self.led1color = led1color
        self.led2 = led2
        self.led2color = led2color
        self.playsound = playsound

        self.led1_pub = rospy.Publisher(
            "/mobile_base/commands/led1", Led, queue_size=1)
        self.led2_pub = rospy.Publisher(
            "/mobile_base/commands/led2", Led, queue_size=1)
        self.sound_pub = rospy.Publisher(
            '/mobile_base/commands/sound', Sound, queue_size=1)

        

    def execute(self, userdata):

        if self.playsound:
            self.sound_pub.publish(Sound(0))
        if self.led1:
            self.led1_pub.publish(self.led1color)
        if self.led2:
            self.led2_pub.publish(self.led2color)

        if  self.led1 or self.led2:
            rospy.sleep(rospy.Duration(1.5))
            if self.led1:
                self.led1_pub.publish(0)
            if self.led2:
                self.led2_pub.publish(0)

        return "done"

class CheckCompletion(State):
    def __init__(self, backup=True):
        State.__init__(self, outcomes=["completed", "not_completed", "next"])
        self.backup = backup

    def execute(self, userdata):
        global PHASE4_TASK_COMPLETED, CURRENT_CHECKPOINT, PHASE4_BOX_FOUND, PHASE4_GOAL_FOUND

        if START and not rospy.is_shutdown():
            PHASE4_TASK_COMPLETED += 1

            if PHASE4_BOX_FOUND and PHASE4_GOAL_FOUND:
                return "completed"
            else:
                CURRENT_CHECKPOINT += 1

                if self.backup:
                    return "not_completed"
                else:
                    return "next"


class PushBox(State):
    def __init__(self):
        State.__init__(self, outcomes=["done"])

        self.end_goal = MoveBaseGoal()
        self.move_base_client = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction)

    def execute(self, userdata):
        global PHASE4_FACING

        # determine if we should go to box's left or right
        if PHASE4_FACING == "box":
            goToRight = True
        elif PHASE4_FACING == "goal":
            goToRight = False
        
        self.push(goToRight)
        return "done"

    def push(self, toRight):
        global PHASE4_BOX_CHECKPOINT, PHASE4_BOX_X
        global PHASE4_GOAL_CHECKPOINT, PHASE4_GOAL_X  , PHASE4_PUSH_Y, PHASE4_DELTA_X

        # turn based on toRight
        if toRight:
            Turn(0).execute(None)
        else:
            Turn(180).execute(None)

        # calculate moving distance
        if not toRight:
            dis = abs(PHASE4_BOX_X - PHASE4_GOAL_X) + 0.8 - PHASE4_DELTA_X
        else:
            dis = 0.8 - PHASE4_DELTA_X
        MoveBaseGo(dis).execute(None)

        # turn 90
        Turn(90).execute(None)

        # go forward
        # if not toRight:
        #     MoveBaseGo(1.05).execute(None)
        # else:
        if not toRight:
            MoveBaseGo(PHASE4_PUSH_Y + 0.4).execute(None)
        else:
            MoveBaseGo(PHASE4_PUSH_Y + 0.25).execute(None)
        # MoveBaseGo(PHASE4_PUSH_Y + 0.25).execute(None)

        # turn based on toRight
        if toRight:
            Turn(180).execute(None)
        else:
            Turn(0).execute(None)

        # Push
        dis = abs(PHASE4_BOX_X - PHASE4_GOAL_X) + 0.8 - 0.4
        # MoveBaseGo(dis).execute(None)
        Translate(dis, 0.5).execute(None)

        # Signal when done
        Signal4(True, 1, True, 3).execute(None)

class MoveBaseUsingOdom(State):
    def __init__(self):
        State.__init__(self, outcomes=["done"])

        self.move_base_client = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction)

    def execute(self, userdata):
        global PHASE4_EXIT_GOAL
        while not rospy.is_shutdown():
            self.move_base_client.send_goal_and_wait(PHASE4_EXIT_GOAL)
            return "done"

def image_callback(msg):
    global BRIDGE, IMAGE

    IMAGE = BRIDGE.imgmsg_to_cv2(msg, desired_encoding='bgr8')

def odom_callback(msg):
    global TB_POSE

    TB_POSE = msg.pose.pose

def green_callback(msg):
    global GREEN_FOUND

    GREEN_FOUND = msg.data

def marker_callback(msg):
    global PHASE4_START_ADJUST, PHASE4_FINISHED_ADJUST
    if PHASE4_START_ADJUST:
        if len(msg.markers) <= 0:
            PHASE4_FINISHED_ADJUST = True
        __, __, angles, position, __ = decompose_matrix(numpify(msg.markers[0].pose.pose))
        target_angle = angles[2] + np.pi


if __name__ == "__main__":
    rospy.init_node('comp5')

    Kp = rospy.get_param("~Kp", 1.0 / 200.0)
    Kd = rospy.get_param("~Kd", 1.0 / 400.0)
    Ki = rospy.get_param("~Ki", 0)
    linear_vel = rospy.get_param("~linear_vel", 0.5)

    white_max_h = rospy.get_param("~white_max_h", 255)
    white_max_s = rospy.get_param("~white_max_s", 72)
    white_max_v = rospy.get_param("~white_max_v", 256)

    white_min_h = rospy.get_param("~white_min_h", 0)
    white_min_s = rospy.get_param("~white_min_s", 0)
    white_min_v = rospy.get_param("~white_min_v", 230)

    red_max_h = rospy.get_param("~red_max_h", 226.8)
    red_max_s = rospy.get_param("~red_max_s", 295.2)
    red_max_v = rospy.get_param("~red_max_v", 256)

    red_min_h = rospy.get_param("~red_min_h", 0)
    red_min_s = rospy.get_param("~red_min_s", 64.8)
    red_min_v = rospy.get_param("~red_min_v", 194)

    red_timeout = rospy.Duration(rospy.get_param("~red_timeout", 0.5))

    red_area_threshold = rospy.get_param("~red_area_threshold", 25000)

    rospy.Subscriber("/joy", Joy, callback=joy_callback)
    rospy.Subscriber("odom", Odometry, callback=odom_callback)
    rospy.Subscriber('/usb_cam/image_raw', Image, callback=image_callback)
    rospy.Subscriber('hasShape2', Bool, callback=green_callback)
    rospy.Subscriber('ar_pose_marker_base', AlvarMarkers, callback=marker_callback)

    srv = Server(Comp5Config, dr_callback)

    sm = StateMachine(outcomes=['success', 'failure'])
    with sm:
        StateMachine.add("Wait", WaitForButton(),
            transitions={'pressed': 'Phase1', 'exit': 'failure'})
            # transitions={'pressed': 'Phase4', 'exit': 'failure'})
                         

        StateMachine.add("Ending", Signal4(True,1, True,1),
                         transitions={"done": "success"})

        # # Phase 1 sub state
        phase1_sm = StateMachine(outcomes=['success', 'failure', 'exit'])
        with phase1_sm:
            StateMachine.add("Finding1", FollowLine("1.0"), transitions={
                             "see_red": "Turn11", "failure": "failure", "exit": "exit", "see_nothing": "failure", "see_long_red": "failure","all_done":"success"})
            StateMachine.add("Turn11", Turn(90), transitions={
                             "success": "Count1", "failure": "failure", "exit": "exit"})  # turn left 90 degrees
            StateMachine.add("Count1", DepthCount(), transitions={
                             "success": "MakeSignal1", "failure": "failure", "exit": "exit"})
            StateMachine.add("MakeSignal1", Signal1(), transitions={
                             "success": "Turn12", "failure": "failure", "exit": "exit"})
            StateMachine.add("Turn12", Turn(0), transitions={
                "success": "success", "failure": "failure", "exit": "exit"})  # turn right 90 degrees
        StateMachine.add("Phase1", phase1_sm, transitions={
                         'success': 'Phase2', 'failure': 'failure', 'exit': 'Wait'})

        # Phase 2 sub state
        phase2_sm = StateMachine(outcomes=['success', 'failure', 'exit'])
        with phase2_sm:
            StateMachine.add("Finding2", FollowLine("2.0"), transitions={
                "see_red": "Turn21", "failure": "failure", "exit": "exit", "see_nothing": "failure", "see_long_red": "failure","all_done":"success"})
            StateMachine.add("Turn21", Turn(180), transitions={
                             "success": "FollowToEnd", "failure": "failure", "exit": "exit"})
            StateMachine.add("FollowToEnd", FollowLine("2.1"), transitions={
                             "see_red": "TurnLeftAbit", 'failure': "FindGreenShape", "exit": "exit", "see_nothing": "failure", "see_long_red": "failure","all_done":"success"})
            StateMachine.add("TurnLeftAbit", Turn(-100), transitions={
                             "success": "Backup", "failure": "failure", "exit": "exit"})
            StateMachine.add("Backup", Translate(
                distance=0.05, linear=-0.2), transitions={"success": "FindGreenShape","failure": "failure", "exit": "exit"})
            StateMachine.add("FindGreenShape", FindGreen(), transitions={
                "success": "TurnBack",  "failure": "failure", "exit": "exit"})
            StateMachine.add("TurnBack", Turn(90), transitions={
                "success": "MoveForward", "failure": "failure", "exit": "exit"})
            StateMachine.add("MoveForward", FollowLine("2.2"), transitions={
                "see_red": "MoveStraightToPoint", "failure": "failure", "exit": "exit", "see_nothing": "failure", "see_long_red": "failure","all_done":"success"})
            StateMachine.add("MoveStraightToPoint", Translate(0.35, 0.2), transitions={
                "success": "Turn22","failure": "failure", "exit": "exit"})
            StateMachine.add("Turn22", Turn(90), transitions={
                "success": "success", "failure": "failure", "exit": "exit"})  # turn left 90
        StateMachine.add("Phase2", phase2_sm, transitions={
            'success': 'Phase4', 'failure': 'failure', 'exit': 'Wait'})

        # Phase 3 sub state
        phase3_sm = StateMachine(outcomes=['success', 'failure', 'exit'])
        with phase3_sm:
            
            StateMachine.add("Finding3", FollowLine("3.1"), transitions={
                "see_red": "Turn31", "failure": "failure", "exit": "exit", "see_nothing": "failure", "see_long_red": "failure", "all_done":"success"})
            StateMachine.add("Turn31", Turn(0), transitions={
                             "success": "BackupALittle", "failure": "failure", "exit": "exit"})  # turn left 90
            StateMachine.add("BackupALittle", Translate(0.10, -0.2), transitions={
                "success": "CheckShape","failure": "failure", "exit": "exit"})
            StateMachine.add("CheckShape", CheckShape2(), transitions={
                             "matched": "Signal3", "failure": "ForwardALittle", "exit": "exit"})
            StateMachine.add("Signal3", Signal3(), transitions={
                             "success": "ForwardALittle", "failure": "failure", "exit": "exit"})
            StateMachine.add("ForwardALittle", Translate(0.10, 0.2), transitions={
                "success": "TurnRight","failure": "failure", "exit": "exit"})
            StateMachine.add("TurnRight", Turn(-90), transitions={
                             "success": "Finding3", "failure": "failure", "exit": "exit"})

        StateMachine.add("Phase3", phase3_sm, transitions={
            'success': 'Ending', 'failure': 'failure', 'exit': 'Wait'})

        phase4_sm = StateMachine(outcomes=['success', 'failure', 'exit'])

        move_list = {
            "point8": [Turn(90), MoveBaseGo(1.05), Turn(0), Translate(0.2, -0.2)],
            "point7": [Turn(90), MoveBaseGo(0.1), Turn(180), MoveBaseGo(0.6), Turn(-90)],
            "point6": [Turn(180), MoveBaseGo(0.8), Turn(-90)],
            # "look_for_box": [Turn(180), MoveBaseGo(1.2), Turn(90), MoveBaseGo(0.6), Turn(0), Translate(0.1)],
            # "point1": [Translate(0.1,0.2), Turn(-90), MoveBaseGo(1.2), Turn(90)],
            "point1": [Turn(180), MoveBaseGo(1.15), Turn(90), Translate(0.3)],
            "point2": [Turn(0), MoveBaseGo(0.75), Turn(90)],
            "point3": [Turn(0), MoveBaseGo(0.8), Turn(90)],
            "point4": [Turn(0), MoveBaseGo(0.85), Turn(90)],
            "point5": [Turn(0), MoveBaseGo(0.8), Turn(90)],
            "exit": [Turn(-90), MoveBaseGo(0.7), Turn(-90)]
        }

        # park_distance =       [0.6,       0.5,       0.5,     0.5 ,       0.5,       0.5,   0.5,      0.5,      0.5]

        # checkpoint_sequence = ["point8", "point7", "point6", "look_for_box" ,"point1", "point2","point3","point4","point5", "exit"]

        # checkpoint_backup_angle = [0, -90, -90, 90, 90, 90,90,90,90,90]

        park_distance =       [0.6,       0.5,       0.5,       0.5,       0.5,   0.5,      0.5,      0.5]
        checkpoint_sequence = ["point8", "point7", "point6" ,"point1", "point2","point3","point4","point5", "exit"]
        checkpoint_backup_angle = [0, -90, -90, 90, 90,90,90,90,90]

        with phase4_sm:
            i = 0

            StateMachine.add("Finding4", FollowLine("4.1"), transitions={	  
                "see_long_red": "MoveForward", "see_nothing": "failure", "see_red": "failure", "failure": "failure", "exit": "exit"	,"all_done":"success"
            })
            StateMachine.add("MoveForward", Translate(distance=0.65, linear=0.2), transitions={
                "success": "Turn41",  "failure": "failure", "exit": "exit"
            })

            StateMachine.add("Turn41", Turn(135), transitions={
                "success": "FollowRamp", "failure": "failure", "exit": "exit"
            })

            StateMachine.add("FollowRamp", FollowLine("4.2"), transitions={
                "see_nothing": checkpoint_sequence[0] + "-0", "see_long_red": "failure", "see_red": "failure", "failure": "failure", "exit": "exit" ,"all_done":"success"
            })

            for i in xrange(len(checkpoint_sequence)):
                moves_to_point = move_list[checkpoint_sequence[i]]
                for j in xrange(len(moves_to_point)):
                    
                    name = checkpoint_sequence[i] + "-" + str(j)
                    next_state_name = None

                    if j == len(moves_to_point) - 1:
                        if i < len(checkpoint_sequence) - 1:
                            next_state_name = checkpoint_sequence[i + 1] + "-" + str(0)

                            StateMachine.add(name, moves_to_point[j], transitions={
                                "success": checkpoint_sequence[i] + "-" + "ParkNext", "failure": "failure", "exit": "exit"
                            })
                            StateMachine.add(checkpoint_sequence[i] + "-" + "ParkNext", ParkNext(checkpoint_sequence[i]), transitions={
                                "see_shape": checkpoint_sequence[i] + "-" + "MatchShape", "see_AR_goal": checkpoint_sequence[i] + "-" + "SignalARGoal", "see_AR_box": checkpoint_sequence[i] + "-" + "SignalARBox", "find_nothing": checkpoint_sequence[i] + "-" + "CheckCompletionNoBackup"
                            })

                            StateMachine.add(checkpoint_sequence[i] + "-" + "ParkShape", MoveBaseGo(park_distance[i]), transitions={
                                "success": checkpoint_sequence[i] + "-" + "SignalShape", "failure": "failure", "exit": "exit"
                            })
                            
                            StateMachine.add(checkpoint_sequence[i] + "-" + "TurnBeforeBackup", Turn(checkpoint_backup_angle[i]),transitions={"success": checkpoint_sequence[i] + "-" + "Moveback", "failure": "failure", "exit": "exit"} )

                            StateMachine.add(checkpoint_sequence[i] + "-" + "Moveback", Translate(park_distance[i],-0.2), transitions={
                                "success": next_state_name, "failure": "failure", "exit": "exit"
                            })
                            StateMachine.add(checkpoint_sequence[i] + "-" + "MatchShape", CheckShape(), transitions={
                                "matched": checkpoint_sequence[i] + "-" + "SignalShapeBeforePark", "failure": checkpoint_sequence[i] + "-" + "CheckCompletionNoBackup", "exit": "exit"
                            })

                            StateMachine.add(checkpoint_sequence[i] + "-" + "SignalARGoal", Signal4(True, 1, True, 0), transitions={
                                "done": checkpoint_sequence[i] + "-" + "CheckCompletionNoBackup"
                            })

                            StateMachine.add(checkpoint_sequence[i] + "-" + "SignalARBox", Signal4(True, 3, True, 0), transitions={
                                "done": checkpoint_sequence[i] + "-" + "CheckCompletionNoBackup"
                            })

                            StateMachine.add(checkpoint_sequence[i] + "-" + "SignalShapeBeforePark", Signal4(True, 2, True, 0), transitions={
                                "done": checkpoint_sequence[i] + "-" + "ParkShape"
                            })

                            StateMachine.add(checkpoint_sequence[i] + "-" + "SignalShape", Signal4(True, 2, True, 1), transitions={
                                "done": checkpoint_sequence[i] + "-" + "CheckCompletion"
                            })

                            StateMachine.add(checkpoint_sequence[i] + "-" + "CheckCompletion", CheckCompletion(True), transitions={
                                "completed": next_state_name, "not_completed": checkpoint_sequence[i] + "-" + "TurnBeforeBackup", "next": next_state_name
                            })

                            StateMachine.add(checkpoint_sequence[i] + "-" + "CheckCompletionNoBackup", CheckCompletion(False), transitions={
                                "completed": "Push_Box", "not_completed": checkpoint_sequence[i] + "-" + "Moveback", "next": next_state_name
                            })
                        elif i == len(checkpoint_sequence) -1: # last move of last point
                            StateMachine.add(name, moves_to_point[j], transitions={
                                "success": "success", "failure": "failure", "exit": "exit"
                            })
                    elif j < len(moves_to_point) - 1:
                        next_state_name = checkpoint_sequence[i] + "-" + str(j + 1)

                        StateMachine.add(name, moves_to_point[j], transitions={
                            "success": next_state_name, "failure": "failure", "exit": "exit"
                        })
            StateMachine.add("Push_Box", PushBox() , transitions={"done":"Push_Box_Move-1"})
            StateMachine.add("Push_Box_Move-1",Translate(0.2) , transitions={"success": "Push_Box_Move-2", "failure": "failure", "exit": "exit"})
            StateMachine.add("Push_Box_Move-2", Turn(-90), transitions={ "success": "Push_Box_Move-3", "failure": "failure", "exit": "exit"})
            StateMachine.add("Push_Box_Move-3", Translate(0.5, 0.5),transitions={"success": "Push_Box_Go_To_End", "failure": "failure", "exit": "exit"})
            StateMachine.add("Push_Box_Go_To_End", MoveBaseUsingOdom() ,transitions={"done": "exit-0"})

            StateMachine.add("ForwardUntilWhite", Translate(),
                                        transitions={"success": "success"}) 

        StateMachine.add("Phase4", phase4_sm, transitions={
                         'success': 'Phase3', 'failure': 'failure', 'exit': 'Wait'})

        

    outcome = sm.execute()

