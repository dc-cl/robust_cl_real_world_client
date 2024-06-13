import numpy as np
from math import atan2, cos, sin, pi
import parameters

from DR import Robot
from scipy.optimize import minimize_scalar

R_1 = parameters.R_1

RB = parameters.MAESUREMENT_RANGE_BOUND
BB = parameters.MAESUREMENT_BEARING_BOUND


class Robot_CI(Robot):

    def __init__(self, X, _id, NUM_ROBOTS, flag, LANDMARK_POS) -> None:
        '''
        construction
        ----
        :param: X: numpy.array, Init pose
        :param: _id: int, _id of robot
        :param: NUM_ROBOTS: int, the number of robots
        :param: flag: int
        :param: LANDMARK_POS: position of landmarks
        '''
        assert (
            flag < 2, "only the observation model is at least relative position with flag<2, CI could be used")
        Robot.__init__(self, X, _id, NUM_ROBOTS)
        self.flag = flag
        self.measuring = np.zeros(NUM_ROBOTS, dtype=bool)
        self.contain_bias = np.zeros(NUM_ROBOTS, dtype=bool)

    def reset_rela(self):
        '''
        Reset the variables about the relative measurement
        '''

        self.measuring = np.zeros(self.NUM_ROBOTS, dtype=bool)
        self.contain_bias_rela = np.zeros(self.NUM_ROBOTS, dtype=bool)

        if (self.flag == -1):
            self.measure_noise = np.zeros((self.NUM_ROBOTS, 3))
            self.Z = np.zeros((self.NUM_ROBOTS, 3))

    def measurement_rela(self, cla_trues, measure_noises, measure_bias_whether=[]):
        '''
        Each time keeps all measurements and later sends to the corresponding robot

        :param: cla_true: list of class "Robot_true", All GroundTruth about robots
        :param: measure_noises: list of measure_noises
        :param: measure_bias_whether: list of whether the measurement is biased
        观测其他机器人，给量测加上噪声
        '''

        My_true = cla_trues[self._id].X_true.copy()
        # Loop other robots
        for r in range(self.NUM_ROBOTS):
            if (r == self._id):
                continue
            _range, bearing, X2_ = parameters.measurement(
                cla_trues[r].X_true, My_true)
            if _range <= RB and abs(bearing) <= BB:  # self->r
                self.measuring[r] = True

                if (self.flag == -1):
                    self.measure_noise[r] = measure_noises[r, :]
                    self.Z[r] = (X2_ + measure_noises[r, :]).reshape(1, 3)

    def abso_meas_correct(self, count=500):
        '''
        Absolute Measurement update
        :param: count: simulation round, for showing the weight of the M-estimation
        '''
        J = np.matrix([[0, -1], [1, 0]])
        for LM_id in range(self.LANDMARK_NUM):
            if not self.measuring_landmark[LM_id]:
                continue
            if (self.flag == -1):
                H = np.zeros((3, 3))
                Z_cal = np.zeros((3, 1))

            Z_now = (self.Z_landmark[LM_id]).T
            gamma = parameters.rot_mat_2d(self.X[2])
            dp = self.LANDMARK_POS[3*LM_id:3*LM_id+3, 0] - self.X  # (3,)？

            if (self.flag == -1):
                H_tilde = np.eye(3)
                H_tilde[0:2, 2] = np.array(J @ np.array([dp[0:2]]).T)[:, 0]

                H[3*LM_id:3*LM_id+3, :] = gamma @ -H_tilde
                Z_cal = (gamma @ dp).reshape(3, 1)
                R2 = (R_ALL_1_LANDMARK[3*LM_id:3 *
                      LM_id+3, 3*LM_id:3*LM_id+3])**2

            v = Z_now - Z_cal
            if (self.flag == 0 and abs(v[1, 0]) > 4):
                if (v[1, 0] > 0):
                    v[1, 0] = v[1, 0] - 2*pi
                else:
                    v[1, 0] = v[1, 0] + 2*pi
            sigma_invention = H @ self.P @ H.T + R2

            sigma = None
            try:
                sigma = np.linalg.inv(np.array(sigma_invention))
            except np.linalg.LinAlgError:
                continue
            K_i = self.P @ H.T @ sigma

            self.X = self.X + np.array(K_i @ v).reshape(3,)
            Coeff = np.eye(3) - K_i @ H
            self.P = Coeff @ self.P

            self.X_prediction = self.X.copy()
            self.P_prediction = self.P.copy()
