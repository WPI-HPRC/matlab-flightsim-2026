# This file is just like me
# I need gf not gnc

# Basically for a run, load calibration data, load sensor data, run the EKF, then output to a csv file


import numpy as np
import pandas as pd
import os
import estimator
import utils
from scipy.spatial.transform import Rotation as R

if __name__ == "__main__":

    calibration_profile = os.path.join("Replay", "calibration_profiles", "board_perfect")
    run_name = os.path.join("Replay", "runs", "simulations", "run_0")

    # Load calibration data into matrices and vectors
    gyro_A = pd.read_csv(os.path.join(calibration_profile, "gyro_A.csv")).to_numpy()
    gyro_b = pd.read_csv(os.path.join(calibration_profile, "gyro_b.csv")).to_numpy()

    accel_A = pd.read_csv(os.path.join(calibration_profile, "accel_A.csv")).to_numpy()
    accel_b = pd.read_csv(os.path.join(calibration_profile, "accel_b.csv")).to_numpy()

    mag_A = pd.read_csv(os.path.join(calibration_profile, "mag_A.csv")).to_numpy()
    mag_b = pd.read_csv(os.path.join(calibration_profile, "mag_b.csv")).to_numpy()


    gps_data = pd.read_csv(os.path.join(run_name, "gps_data.csv"))
    imu_data = pd.read_csv(os.path.join(run_name, "imu_data.csv"))
    mag_data = pd.read_csv(os.path.join(run_name, "mag_data.csv"))

    dynamics = pd.read_csv(os.path.join(run_name, "dynamics.csv"))




    # Basically just GPS update position
    inertial_estimator = estimator.BasicEstimator()
    perfect_estimator = estimator.BasicEstimator()
    

    # How to reconcile with differently spaced data:
    # Join the pandas dataframe
    # Keep the extra timesteps
    # If the timestep difference from sensor to sensor is < epsilon, then combine them into one row

    # Initialize using the 5th timestep
    # TODO: Using gravity ned, not normal force, need to switch over, mult by -1
    init_ts = 5
    inertial_estimator.initialize_position(gps_data.loc[init_ts, ["gps_lat", "gps_lon", "gps_alt"]].to_numpy())
    perfect_estimator.initialize_position(gps_data.loc[init_ts, ["gps_lat", "gps_lon", "gps_alt"]].to_numpy())

    normal_e = imu_data.loc[init_ts, ["g_e_x", "g_e_y", "g_e_z"]].to_numpy(copy=True)
    normal_e[2] *= -1
    inertial_estimator.initialize_orientation(imu_data.loc[init_ts, ["imu_a_x", "imu_a_y", "imu_a_z"]].to_numpy(copy=True),
                                          normal_e,
                                          mag_data.loc[init_ts, ["mag_x", "mag_y", "mag_z"]].to_numpy(copy=True),
                                          mag_data.loc[init_ts, ["m_e_x", "m_e_y", "m_e_z"]].to_numpy(copy=True))
    perfect_estimator.orientation = R.from_matrix(np.array(dynamics.loc[init_ts, ['R_BT_00', 'R_BT_01', 'R_BT_02', 'R_BT_10', 'R_BT_11', 'R_BT_12', 'R_BT_20', 'R_BT_21', 'R_BT_22']]).reshape((3, 3)))


    prev_time = imu_data.loc[init_ts, ["imu_time"]].to_numpy()

    highest_alt = -np.inf
    
    for i, time in enumerate(gps_data["gps_time"][init_ts:], start=init_ts):
        R_BT = R.from_matrix(np.array(dynamics.loc[i, ['R_BT_00', 'R_BT_01', 'R_BT_02', 'R_BT_10', 'R_BT_11', 'R_BT_12', 'R_BT_20', 'R_BT_21', 'R_BT_22']]).reshape((3, 3)))
        print(f"Time: {time}. i: {i}")

        gyro_data = imu_data.loc[i, ["imu_g_x", "imu_g_y", "imu_g_z"]].to_numpy(copy=True)
        true_gyro = (gyro_A @ gyro_data + gyro_b).reshape((3,))
        accel_data = imu_data.loc[i, ["imu_a_x", "imu_a_y", "imu_a_z"]].to_numpy(copy=True)
        true_accel = (accel_A @ accel_data + accel_b).reshape((3,))
        g_e = imu_data.loc[i, ["g_e_x", "g_e_y", "g_e_z"]].to_numpy(copy=True)


        # mag_data = mag_data.loc[i, ["mag_x", "mag_y", "mag"]]


        gps_lla = gps_data.loc[i, ["gps_lat", "gps_lon", "gps_alt"]].to_numpy()

        dt = time - prev_time
        perfect_estimator.set_orientation(R_BT.inv())
        perfect_estimator.update_position(gps_lla)
        inertial_estimator.gyro_prop(true_gyro, dt)
        inertial_estimator.accel_prop(true_accel, g_e, dt)

        if perfect_estimator.get_pos_lla()[2] > highest_alt:
            highest_time = time
            highest_i = i
            highest_alt = perfect_estimator.get_pos_lla()[2]
            highest_perfect_ned = perfect_estimator.get_pos_ned().copy()
            highest_estimator_ned = inertial_estimator.get_pos_ned().copy()

        inertial_estimator.log_data(time)
        perfect_estimator.log_data(time)


        prev_time = time


        # TODO figure out the time steps stuff:
        # Whatever sensor data is in that time step
        # Run that throiugh the ekf

    highest_error = highest_perfect_ned - highest_estimator_ned
    ate = highest_error[2]
    xte = np.hypot(highest_error[0], highest_error[1])
    print(f"Highest time: {highest_time}. i: {highest_i}. Alt: {highest_alt}. ate: {ate}. xte: {xte}")

    
    inertial_estimator.save_log(os.path.join(run_name, "inertial_estimator_log.csv"))
    perfect_estimator.save_log(os.path.join(run_name, "perfect_estimator_log.csv"))


    # visualizations.plot_orientation(orientation_log, position_ned_log, save_path="something.jpg")


