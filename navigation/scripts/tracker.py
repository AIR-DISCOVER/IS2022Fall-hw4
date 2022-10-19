#!/usr/bin/env python3
import rospy

import tf2_ros
import tf_conversions

from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalID

import numpy as np
from math import pi, hypot, atan2, cos, sin

from pid import PID, Angle_PID
from reference_path import ReferencePath

class router:
    def __init__(self) -> None:

        rospy.set_param("/navigation/dis_disx_k", 1.0)
        rospy.set_param("/navigation/dis_disy_k", 1.0)
        rospy.set_param("/navigation/dis_angle_k", 0.5)
        rospy.set_param("/navigation/pre_point", 2)
        rospy.set_param("/navigation/window_size", 20)
        self.update_param()

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

        while not rospy.is_shutdown():
            try:
                self.tfBuffer.lookup_transform('map', 'base_link', rospy.Time())
                rospy.loginfo("Get transform between map and baselink.")
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logwarn("Can not get transform between map and baselink.")
                rospy.sleep(0.5)

        # current position and yaw w.r.t. map
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.yaw = 0.0

        # target position and yaw w.r.t. map
        self.target_pose_x = 0.0
        self.target_pose_y = 0.0
        self.target_yaw = 0.0

        # 运动的三个自由度pid控制器
        self.pid_xarr = PID(1.5, 0.75, 0, 0.9, -0.9, 0.1)
        self.pid_yarr = PID(1.5, 0.75, 0, 0.9, -0.9, 0.1)
        self.pid_thetaarr = Angle_PID(3.0, 0.8, 0, 0.9, -0.9, 0.1)

        # 目标点的距离和角度容限
        self.tor_posi = 0.05
        self.tor_ang = 0.1

        self.rate = 10  # 控制器频率
        self.tracking_goal_lock = False # 进程锁
        self.reference_path = []        # 参考路径
        self.cmd_vel_puber = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.global_path_suber = rospy.Subscriber("/move_base/GlobalPlanner/plan", Path, self.globalPathCallback)
        self.cancel_goal_puber = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)

    def update_param(self) -> None:
        """
        :brief:   更新机器人坐标
        :return:  是否获得了机器人坐标
        """
        self.param_dis_disx_k = rospy.get_param("/navigation/dis_disx_k")
        self.param_dis_disy_k = rospy.get_param("/navigation/dis_disy_k")
        self.param_dis_angle_k = rospy.get_param("/navigation/dis_angle_k")
        self.param_pre_point = rospy.get_param("/navigation/pre_point")
        self.param_window_size = rospy.get_param("/navigation/window_size")


    def get_robot_pos(self) -> bool:
        """
        :brief:   更新机器人坐标
        :return:  是否获得了机器人坐标
        """
        try:
            transform = self.tfBuffer.lookup_transform('map', 'base_link', rospy.Time()).transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return False
        quat = [transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w]
        euler = tf_conversions.transformations.euler_from_quaternion(quat)
        self.pose_x = transform.translation.x
        self.pose_y = transform.translation.y
        self.yaw = euler[2]
        return True


    def globalPathCallback(self, msg:Path) -> None:
        """
        :brief:   话题回调函数
        """
        p = msg.poses[-1]
        quat = [p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w]
        euler = tf_conversions.transformations.euler_from_quaternion(quat)

        if hypot(p.pose.position.x-self.pose_x, p.pose.position.y-self.pose_y) > self.tor_posi or abs(self.pid_thetaarr.angle_bias(self.target_yaw, self.yaw)) > self.tor_ang:
            self.update_param()
            yaw_lst = []
            for p in msg.poses:
                quat = [p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w]
                euler = tf_conversions.transformations.euler_from_quaternion(quat)
                yaw_lst.append(euler[2])
            path_np = {
                "x" : np.array([p.pose.position.x for p in msg.poses]),
                "y" : np.array([p.pose.position.y for p in msg.poses]),
                "yaw" : np.array(yaw_lst),
            }
            self.target_pose_x = path_np["x"][-1]
            self.target_pose_y = path_np["y"][-1]
            self.target_yaw = path_np["yaw"][-1]
            self.reference_path = ReferencePath(path_np, pre_point=self.param_pre_point, window_size=self.param_window_size)
            self.tracking_goal_lock = True


    def odom2baselink(self, vx:float, vy:float, yaw:float) -> tuple:
        """
        :brief:  map坐标系下的速度变换到baselink坐标系下的速度
        :param:  vx,vy是map坐标系下的目标速度,yaw是当前ep机器人的朝向
        :return: 指定小车的线速度(vvx,vvy)
        """
        varr = hypot(vx, vy)
        theta = atan2(vy, vx)
        vvx = varr * cos(theta - yaw)
        vvy = varr * sin(theta - yaw)
        return (vvx, vvy)


    def pubcmd(self, vx:float, vy:float, vth:float) -> None:
        """
        :brief: 发布底盘速度指令
        :param: 速度m/s是odom坐标系,角速度rad/s方向逆时针为正
        """
        twist = Twist()
        vvx, vvy = self.odom2baselink(vx, vy, self.yaw)
        tmpv = max(abs(vvx), abs(vvy))
        if 1e-6 < tmpv < 0.1:
            vvx = 0.1 * vvx / tmpv
            vvy = 0.1 * vvy / tmpv
        twist.linear.x = vvx
        twist.linear.y = vvy
        twist.angular.z = vth
        self.cmd_vel_puber.publish(twist)


    def loop(self) -> None:
        """
        :brief: 程序主循环
        :param: 速度m/s是odom坐标系,角速度rad/s方向逆时针为正
        """
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            if self.get_robot_pos():
                if self.tracking_goal_lock:
                    xref, approx = self.reference_path.calc_ref_trajectory([self.pose_x, self.pose_y])
                    dis_x = xref[0] - self.pose_x
                    dis_y = xref[1] - self.pose_y
                    dis_angle = xref[2] - self.yaw
                    if dis_angle > np.pi:
                        dis_angle -= 2.0 * np.pi
                    elif dis_angle < -np.pi:
                        dis_angle += 2.0 * np.pi
                    if approx:
                        if hypot(self.target_pose_x-self.pose_x, self.target_pose_y-self.pose_y) < self.tor_posi and abs(self.pid_thetaarr.angle_bias(self.target_yaw, self.yaw)) < self.tor_ang:
                            self.pubcmd(0.0, 0.0, 0.0)
                            self.tracking_goal_lock = False
                            rospy.loginfo("Reach Goal")
                        else:
                            vx = self.pid_xarr.out(self.target_pose_x, self.pose_x)
                            vy = self.pid_yarr.out(self.target_pose_y, self.pose_y)
                            vth = self.pid_thetaarr.out(self.target_yaw, self.yaw)
                            self.pubcmd(vx, vy, vth)
                    else:
                        self.pubcmd(self.param_dis_disx_k*dis_x, self.param_dis_disy_k*dis_y, self.param_dis_angle_k*dis_angle)
            else:
                self.pubcmd(0.0, 0.0, 0.0)
                rospy.logwarn("can not get trans between map and baselink")
            r.sleep()


if __name__ == "__main__":
    rospy.init_node('shadow_router', anonymous=True)
    rospy.loginfo(">>>>>>>>>>>>>>>>>>>")
    rter = router()
    rter.loop()
    rospy.spin()


