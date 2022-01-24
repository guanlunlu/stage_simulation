#!/usr/bin/env python3
import rospy
import math
import tf
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
pi = math.pi

class state_vector:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta
    
    def matrix(self):
        return np.mat([[self.x], 
                       [self.y],
                       [self.theta]])

    def show(self):
        return ((self.x, self.y, self.theta))

class correction:
    def __init__(self):
        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odomCallback)
        self.scan_sub = rospy.Subscriber("base_scan", LaserScan, self.scanCallback)
        self.scan_pub = rospy.Publisher("scan", LaserScan, queue_size = 10)
        self.odom_init = 0
        self.odom_past = state_vector(0,0,0)

        self.bf_real = state_vector(0,0,0)

    def odomCallback(self, data):
        if self.tf_listener.canTransform('odom', 'base_footprint', rospy.Time(0)):
            bf_odom = self.tf_listener.lookupTransform('odom', 'base_footprint', rospy.Time(0))
            cur_x = bf_odom[0][0]
            cur_y = bf_odom[0][1]
            (roll, pitch, yaw) = euler_from_quaternion(bf_odom[1])
            cur_theta = yaw

            if self.odom_init:
                d_x = cur_x - self.odom_past.x
                d_y = cur_y - self.odom_past.y
                d_theta = self.theta_error_signed(self.odom_past.theta, cur_theta)
                # print("d_x, d_y", d_x,d_y)

                tf_angle = self.theta_convert(cur_theta)
                vec_x = d_x * math.cos(-tf_angle) + d_y * -math.sin(-tf_angle)
                vec_y = d_x * math.sin(-tf_angle) + d_y * math.cos(-tf_angle)
                
                ############################
                # for stage simulation bug #
                vec_y *= -1                #
                ############################
                d_x_ = vec_x * math.cos(tf_angle) + vec_y * -math.sin(tf_angle)
                d_y_ = vec_x * math.sin(tf_angle) + vec_y * math.cos(tf_angle)
                # print("d_x_, d_y_", d_x_,d_y_)
                self.bf_real.x += d_x_
                self.bf_real.y += d_y_
                self.bf_real.theta += d_theta
                # print(self.bf_real.x, self.bf_real.y, self.bf_real.theta)
                self.transform_publish(self.bf_real)
                self.odom_past.x = cur_x
                self.odom_past.y = cur_y
                self.odom_past.theta = cur_theta
            else:
                self.odom_past.x = cur_x
                self.odom_past.y = cur_y
                self.odom_past.theta = cur_theta
                self.State_past = state_vector(cur_x, cur_y, cur_theta)
                self.bf_real = state_vector(cur_x, cur_y, cur_theta)

                self.transform_publish(self.State_past)
                self.odom_init = 1
                # print("Initial pose at ", (cur_x, cur_y, cur_theta))

    def scanCallback(self, msg):
        msg.header.frame_id = "true_base_footprint"
        self.scan_pub.publish(msg)

    def transform_publish(self, bf_state):
        rate = rospy.Rate(50)
        quat = quaternion_from_euler(0,0,bf_state.theta)
        pose = (bf_state.x, bf_state.y, 0)
        self.tf_broadcaster.sendTransform(pose, quat, rospy.Time.now(), "true_base_footprint", "odom")
        rate.sleep()

    def theta_error_signed(self, theta_start, theta_final):
        curPos_vx = math.cos(theta_start)
        curPos_vy = math.sin(theta_start)
        goalPos_vx = math.cos(theta_final)
        goalPos_vy = math.sin(theta_final)
        dot = round(curPos_vx * goalPos_vx + curPos_vy * goalPos_vy, 5)
        theta_err = math.acos(dot)

        if abs(self.theta_convert(theta_start + theta_err) - theta_final) > 0.001:
            theta_err *= -1
        return theta_err

    def theta_convert(self, input):
        # convert rad domain to [-pi, pi]
        pi = math.pi
        if input >=0:
            input = math.fmod(input, 2*pi)
            if input > pi:
                input -= 2*pi
                output = input
            else:
                output = input
        else:
            input *= -1
            input = math.fmod(input, 2*pi)
            if input > pi:
                input -= 2*pi
                output = input*-1
            else:
                output = input*-1
        return output


if __name__ == '__main__':
    rospy.init_node('stage_odom_correction', anonymous = False)
    correction = correction()
    rospy.spin()
