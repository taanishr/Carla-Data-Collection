import numpy as np
import math

class Calib:
    def __init__(self, height, width):
        self.height = height
        self.width = width

        self.ravel_mode = 'C'
        self.P0 = self.generate_intrinsic_mat()
        self.P0 = np.column_stack((self.P0, np.array([0, 0, 0])))
        self.P0 = np.ravel(self.P0, order=self.ravel_mode)

        self.R0 = np.identity(3)
        self.TR_velodyne = np.array([[0, -1, 0],
                                    [0, 0, -1],
                                    [1, 0, 0]])

        self.TR_velodyne = np.column_stack((self.TR_velodyne, np.array([0, 0, 0])))
        self.TR_imu_to_velo = np.identity(3)
        self. TR_imu_to_velo = np.column_stack((self.TR_imu_to_velo, np.array([0, 0, 0])))

    def generate_intrinsic_mat(self):
        window_height = self.height
        window_width = self.width
        window_height_half = window_height / 2
        window_width_half = window_width / 2

        k = np.identity(3)
        k[0, 2] = window_width_half
        k[1, 2] = window_height_half

        f = window_width / \
            (2.0 * math.tan(90.0 * math.pi / 360.0))

        k[0, 0] = k[1, 1] = f

        return k

    def write_flat(self, f, name, arr):
        f.write("{}: {}\n".format(name, ' '.join(
            map(str, arr.flatten(self.ravel_mode).squeeze()))))

    def save_calib_matrix(self, filename):
        with open(filename, 'w') as f:
            for i in range(4):  # Avod expects all 4 P-matrices even though we only use the first
                self.write_flat(f, "P" + str(i), self.P0)
            self.write_flat(f, "R0_rect", self.R0)
            self.write_flat(f, "Tr_velo_to_cam", self.TR_velodyne)
            self.write_flat(f, "TR_imu_to_velo", self.TR_imu_to_velo)