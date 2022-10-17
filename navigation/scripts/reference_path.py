import numpy as np

class ReferencePath:
    def __init__(self, path, pre_point=15, window_size=20):
        self.ncourse = len(path["x"])
        self.refer_path = np.zeros((self.ncourse, 3))
        self.refer_path[:, 0] = path["x"]
        self.refer_path[:, 1] = path["y"]
        self.refer_path[:, 2] = path["yaw"]
        self.index = 0
        self.pre_point = pre_point
        self.window_size = window_size

    def update_param(self, pre_point, window_size):
        self.pre_point = pre_point
        self.window_size = window_size

    def calc_track_error(self, x, y):
        idx_end = min(self.index+self.window_size, self.refer_path.shape[0]-1)
        d_x = self.refer_path[self.index:idx_end, 0] - x
        d_y = self.refer_path[self.index:idx_end, 1] - y
        dis = np.hypot(d_x, d_y)
        idx = np.argmin(dis)
        self.index = self.index+idx
        return self.index

    def calc_ref_trajectory(self, robot_state):
        index = self.calc_track_error(robot_state[0], robot_state[1])

        xref = np.zeros(3)
        if (index + self.pre_point) < self.ncourse:
            xref[0] = self.refer_path[index + self.pre_point, 0]
            xref[1] = self.refer_path[index + self.pre_point, 1]
            xref[2] = self.refer_path[index + self.pre_point, 2]
            approx = False
        else:
            xref[0] = self.refer_path[self.ncourse - 1, 0]
            xref[1] = self.refer_path[self.ncourse - 1, 1]
            xref[2] = self.refer_path[self.ncourse - 1, 2]
            approx = True

        return xref, approx
