# Purpose: This file takes the output dynamics.csv file and puts into a standard csv format for each sensor
# Basically to align the dynamics output with what the EKF Would see in the sensors
# TODO eventually add noise and stuff, right now just the basics

import pandas as pd
import numpy as np
from scipy.spatial.transform import Rotation as R
import os

all_matlab_data = pd.read_csv(os.path.join("Replay", "runs", "simulations", "dynamics.csv"))

gps_data = pd.DataFrame(columns=["gps_time", "gps_lat", "gps_lon", "gps_alt", "gps_vn", "gps_ve", "gps_vu"])

imu_data = pd.DataFrame(columns=["imu_time", "imu_a_x", "imu_a_y", "imu_a_z", "imu_g_x", "imu_g_y", "imu_g_z"])

baro_data = pd.DataFrame(columns=["baro_time", "baro_alt"])



for i, time in enumerate(all_matlab_data['sim_time'][0:1000]):
    # print(f"Time: {time}")
    r_ecef = np.array(all_matlab_data.loc[i, ['P_E_0', 'P_E_1', 'P_E_2']])
    v_ecef = np.array(all_matlab_data.loc[i, ['V_E_0', 'V_E_1', 'V_E_2']])
    R_BT = R.from_matrix(np.array(all_matlab_data.loc[i, ['R_BT_00', 'R_BT_01', 'R_BT_02', 'R_BT_10', 'R_BT_11', 'R_BT_12', 'R_BT_20', 'R_BT_21', 'R_BT_22']]).reshape((3, 3)))
    R_TE = R.from_matrix(np.array(all_matlab_data.loc[i, ['R_TE_00', 'R_TE_01', 'R_TE_02', 'R_TE_10', 'R_TE_11', 'R_TE_12', 'R_TE_20', 'R_TE_21', 'R_TE_22']]).reshape((3, 3)))

    v_ned = R_TE.apply(v_ecef)

    r_lla = np.array(all_matlab_data.loc[i, ['LLA_0', 'LLA_1', 'LLA_2']])


    time_ms = i / 10000
    new_row = pd.DataFrame([{
        "gps_time": time,
        "gps_lat": r_lla[0],
        "gps_lon": r_lla[1],
        "gps_alt": r_lla[2],
        "gps_vn": v_ned[0],
        "gps_ve": v_ned[1],
        "gps_vu": -1.0 * v_ned[2],
    }])

    gps_data = pd.concat([gps_data, new_row], ignore_index=True)

    # imu_data = imu_data.append({
    #     "imu_time": time,
    #     "imu_a_x": all_matlab_data.loc[i, "a_body_0"],
    #     "imu_a_y": all_matlab_data.loc[i, "a_body_1"],
    #     "imu_a_z": all_matlab_data.loc[i, "a_body_2"],
    # })

print(gps_data.tail())


