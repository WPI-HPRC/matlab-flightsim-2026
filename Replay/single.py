# This file is just like me
# I need gf not gnc

# Basically for a run, load calibration data, load sensor data, run the EKF, then output to a csv file
import numpy as np
import pandas as pd
import os
import estimator

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





    this_estimator = estimator.BasicEstimator()

    # How to reconcile with differently spaced data:
    # Join the pandas dataframe
    # Keep the extra timesteps
    # If the timestep difference from sensor to sensor is < epsilon, then combine them into one row

    for i, time in enumerate(gps_data["gps_time"]):
        gyro_data = imu_data.loc[i, ["imu_g_x", "imu_g_y", "imu_g_z"]].to_numpy()

        true_gyro = gyro_A * gyro_data + gyro_b

        print(true_gyro)

        # TODO plug in the estimator


        pass

        # TODO figure out the time steps stuff:

        # Whatever sensor data is in that time step
        # Run that throiugh the ekf


    # Output the EKF logged state to a csv file for viz, etc. later
    # Print out the XTE, ATE and other metrics

    



