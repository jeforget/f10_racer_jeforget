#!/usr/bin/env python3

import rospy
import rospkg
import yaml
import math
import numpy as np
import matplotlib.pyplot as plt
import csv
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDrive
from tf.transformations import euler_from_quaternion


class PurePursuit:
    def __init__(self, odom_topic, cmd_topic, goals):

        # set up pubs, subs and stuff
        rospy.init_node('pure_pursuit_controller', anonymous=True)

        self.sub = rospy.Subscriber(odom_topic, Odometry, callback=self.get_current_position)
        self.sub2 = rospy.Subscriber('/race_start', Bool, callback=self.check_start)
        #print("subbed")
        self.pub = rospy.Publisher(cmd_topic, AckermannDrive, queue_size=10)

        self.curr_x = 0
        self.curr_y = 0
        self.curr_yaw = 0

        self.start = False

        # for the plot
        self.traj_x = []
        self.traj_y = []

        # static speed
        self.speed = 1.5

        # wheelbase
        self.l = 0.324

        # index of the current target
        self.curr_target = 0

        # list of points from .yaml file
        self.points = np.array(goals)

        self.goals_size = len(self.points)

    def check_start(self, bool_msg):
        self.start = bool_msg.data

    # from PA2, callback for sub
    def get_current_position(self, odom_msg):
        #print("called func")
        self.curr_x = odom_msg.pose.pose.position.x
        self.curr_y = odom_msg.pose.pose.position.y
        quat = odom_msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        self.curr_yaw = yaw
        self.traj_x.append(self.curr_x)
        self.traj_y.append(self.curr_y)
        return
    
    # also from PA2
    def calc_velocity(self, target_x, target_y):
        # calc distance to the target using euclidean distance formula
        distance = math.sqrt((target_x - self.curr_x) ** 2 + (target_y - self.curr_y) ** 2)
        
        # calc the angle to the target
        target_angle = math.atan2(target_y - self.curr_y, target_x - self.curr_x)
        
        # diff in angle
        angle_diff = target_angle - self.curr_yaw

        return distance, angle_diff
    
    def publish(self, steer):
        #self.curr_target < 1 or self.curr_target == 4 or 5 <= self.curr_target <= 6 or 9 < self.curr_target < 14 or self.curr_target == 27 or self.curr_target > 29:
        msg = AckermannDrive()
        if 11 <= self.curr_target < 13:
            msg.speed = 3.9
        elif self.curr_target == 32:
            msg.speed = 2.0
        elif 14 <= self.curr_target <= 26:
            msg.speed = 2.2
        elif self.curr_target < 2 or self.curr_target > 30:
            msg.speed = 3.0
        elif 27 <= self.curr_target <= 28 or self.curr_target == 3:
            msg.speed = 1.8
        else:
            msg.speed = 2.0
        msg.steering_angle = steer
        self.pub.publish(msg)

    def stop(self):
        msg = AckermannDrive()
        msg.speed = 0.0
        msg.steering_angle = 0.0
        self.pub.publish(msg)

    def calc_pp(self):
        target = self.points[self.curr_target]
        #print(target)
        target_x = target[0]
        target_y = target[1]
        ld, a = self.calc_velocity(target_x, target_y)
        steer = np.arctan((2 * self.l * np.sin(a)) / ld)
        return steer, ld

if __name__ == "__main__":

    rp = rospkg.RosPack()
    path = rp.get_path('f10_racing')

    config_path = path + "/config/params.yaml"
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    
    odom_topic = config["odom_topic"]
    command_topic = config["command_topic"]

    
    goals = []
    idx = 0
    csv_path = path + "/csv/gp_centerline.csv"
    with open(csv_path, 'r') as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            idx += 1
            if row[0] != "x":
                if idx % 100 == 0 or idx == 0 or idx == 1757:
                    goals.append([float(row[0]),float(row[1])])
                elif (100<= idx <= 200) or (1400<= idx <= 1700) or (200<= idx <= 300):
                    if idx % 50 == 0:
                        goals.append([float(row[0]),float(row[1])])
                elif (500<= idx <= 600) or (850<= idx <= 1200):
                    if idx % 30 == 0:
                        goals.append([float(row[0]),float(row[1])])
                elif (1201<= idx <= 1300):
                    if idx % 60 == 0:
                        goals.append([float(row[0]),float(row[1])])

    goals = np.array(goals)
            
    
    pp = PurePursuit(odom_topic, command_topic, goals)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rate.sleep()

        # wait for start
        while pp.start != True:
            continue

        steer, ld = pp.calc_pp()
        #print(f"steer = {steer}")
        #print(f"ld = {ld}")
        #print(f"len_of_points = {pp.goals_size}")
        #print(f"target = {pp.points[pp.curr_target]}")
        #print(f"current coords: {pp.curr_x}, {pp.curr_y}, {pp.curr_yaw}")
        
        # car was going the wrong way, so I just negated it and it worked
        pp.publish(-steer)
        
        # if a goal is reached, move on to the next one
        if ld <= 0.5:
            pp.curr_target += 1

        # stop if all goals are met
        if pp.curr_target >= pp.goals_size:
            pp.stop()
            break

    plt.figure()
    plt.plot(pp.traj_x, pp.traj_y)
    plt.scatter(pp.points[:, 0], pp.points[:, 1])
    plt.show()