'''
Author: jia-yf19
Date: 2022-03-05 00:02:56
LastEditTime: 2022-03-15 21:19:34
Description: Pid控制器
FilePath: /icra_solution/scripts/pid.py
'''

from math import pi

class PID:
    def __init__(self, Kk:float, Ki:float, Kd:float, maxout:float, minout:float, inte_limit:float) -> None:
        self.Kk = Kk
        self.Ki = Ki
        self.Kd = Kd
        self.maxout = maxout
        self.minout = minout
        self.inte_limit = inte_limit
        self.integral = 0.0
        self.last_err = 0.0
    
    def limit(self, a:float, ub:float, lb:float) -> float:
        if a > ub:
            return ub
        elif a < lb:
            return lb
        else:
            return a

    def reset_inte(self) -> None:
        self.integral = 0.0
        self.last_err = 0.0

    def out(self, target:float, current:float) -> float:
        err = target - current
        self.integral += err
        self.integral = self.limit(self.integral, self.inte_limit, -self.inte_limit)
        out = self.Kk * err + self.Ki * self.integral + self.Kd * (err - self.last_err)
        out = self.limit(out, self.maxout, self.minout)
        self.last_err = err
        return out


class Angle_PID(PID):
    def __init__(self, Kk:float, Ki:float, Kd:float, maxout:float, minout:float, inte_limit:float) -> None:
        super().__init__(Kk, Ki, Kd, maxout, minout, inte_limit)

    def angle_bias(self, a1, a2):
        bias = a1 - a2
        if bias > pi:
            bias -= 2.0 * pi
        elif bias < -pi:
            bias += 2.0 * pi
        return bias

    def out(self, target:float, current:float) -> float:
        err = self.angle_bias(target, current)
        self.integral += err
        self.integral = self.limit(self.integral, self.inte_limit, -self.inte_limit)
        out = self.Kk * err + self.Ki * self.integral + self.Kd * (err - self.last_err)
        out = self.limit(out, self.maxout, self.minout)
        self.last_err = err
        return out


if __name__ == "__main__":
    print("PID class")
    # from tf_conversions import transformations
    # quat = transformations.quaternion_from_euler(0.0, 0.0, 1)
    # print(type(quat))
    # print(quat)

    # from geometry_msgs.msg import PoseStamped
    # simple_goal = PoseStamped()
    # print(type(simple_goal.pose.orientation))
    # print(simple_goal.pose.orientation)

    # print("grens")
    # simple_goal.pose.orientation = quat
    # print(simple_goal.pose.orientation)

    from tf_conversions import transformations

    x = 0.0
    y = 0.0
    z = -0.3333654599764396
    w = 0.9427976824826718

    euler = transformations.euler_from_quaternion([x, y, z, w])
    print(euler)
