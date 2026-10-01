import pymap3d as pm
import numpy as np
import utils
from scipy.spatial.transform import Rotation as R
import pandas as pd

class Estimator():
    def __init__(self):
        pass

    def get_orientation(self):
        pass

    def get_position(self):
        pass


# This class is to abstract the EKF or whatever filter with standardized functions




class BasicEstimator(Estimator):
    def __init__(self):
        super().__init__()

        self.state = []

        self.initial_pos_ecef = None
        self.initial_pos_lla = None
        self.initial_dcm_ecef2ned = None
        self.orientation = R.from_matrix(np.eye(3))

        self.pos_ned = np.zeros((3,))
        self.vel_ned = np.zeros((3,))

        self.pos_ned_log = []
        self.orientation_log = []
        self.time_log = []


    def set_orientation(self, orientation):
        self.orientation = orientation


    def gyro_prop(self, unbiased_gyro, dt):
        self.orientation = self.orientation * R.from_rotvec(unbiased_gyro * dt)

    def accel_prop(self, unbiased_accel, g_e, dt):
        # if norm(accel) > 1e-8
        #     v_dot = quat2rotm(quaternion(new_quat')) * accel + g_i;

        #     new_vel = prev_vel + v_dot * dt;

        # else
        #     new_vel = prev_vel;
        # end

        # new_pos = prev_pos + new_vel * dt;
        a_e = self.get_orientation().apply(unbiased_accel) + g_e
        self.vel_ned += a_e * dt
        self.pos_ned += self.vel_ned * dt


    def get_orientation_ned_rpy(self):
        return self.orientation.as_euler("zyx", degrees=True)

    def get_orientation_ned_quat(self):
        return self.orientation.as_quat(scalar_first=True, canonical=True)

    def get_orientation(self):
        return self.orientation

    def get_pos_ned(self):
        return self.pos_ned
    
    def get_pos_ecef(self):
        return self.initial_pos_ecef + self.initial_dcm_ecef2ned.inv().apply(self.pos_ned)

    def get_pos_lla(self):
        return utils.ecef2lla(self.get_pos_ecef())

    def initialize_orientation(self, a_b, a_i, m_b, m_i):
        self.orientation = utils.TRIAD(a_b, a_i, m_b, m_i)

    def ecef2ned_pos(self, pos_ecef):
        return self.initial_dcm_ecef2ned.apply(pos_ecef - self.initial_pos_ecef)


    def initialize_position(self, gps_lla):
        self.initial_pos_ecef = utils.lla2ecef(gps_lla)
        self.initial_pos_lla = gps_lla
        self.initial_dcm_ecef2ned = utils.dcm_ecef2ned(gps_lla)
        self.pos_ned = np.zeros((3,))

    def update_position(self, gps_lla):
        pos_ecef = utils.lla2ecef(gps_lla)
        self.pos_ned = self.ecef2ned_pos(pos_ecef)

    def log_data(self, time):
        self.pos_ned_log.append(self.pos_ned.copy())
        self.orientation_log.append(self.orientation)
        self.time_log.append(time)

    def save_log(self, filename):
        # output_df["est_orientation_quat"] = output_df["est_orientation"].apply(lambda x: x.as_quat(scalar_first=True, canonical=True))
        # # Split each quaternion into seperate columns
        # output_df[["est_q_w", "est_q_x", "est_q_y", "est_q_z"]] = pd.DataFrame(output_df["est_orientation_quat"].to_list())
        # # Split each position into seperate columns
        # output_df[["est_ned_x", "est_ned_y", "est_ned_z"]] = pd.DataFrame(output_df["est_position_ned"].to_list())

        output_df = pd.DataFrame(
            {
                "time": self.time_log,
                "orientation": self.orientation_log,
                "pos_ned": self.pos_ned_log,
            }
        )

        output_df["orientation_quat"] = output_df["orientation"].apply(lambda x: x.as_quat(scalar_first=True, canonical=True))
        output_df[["q_w", "q_x", "q_y", "q_z"]] = pd.DataFrame(output_df["orientation_quat"].to_list())
        output_df[["ned_x", "ned_y", "ned_z"]] = pd.DataFrame(output_df["pos_ned"].to_list())

        output_df = output_df.drop(columns=["orientation", "orientation_quat", "pos_ned"])

        output_df.to_csv(filename, index=False)


