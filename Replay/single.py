# This file is just like me
# I need gf not gnc

# Basically for a run, load calibration data, load sensor data, run the EKF, then output to a csv file
import numpy as np
import pandas as pd

if __name__ == "__main__":

    calibration_profile = ""
    run_name = ""

    # Load calibration data into matrices and vectors
    gyro_A = np.zeros((3, 3))
    gyro_b = np.zeros((3, 1))

    accel_A = np.zeros((3, 3))
    accel_b = np.zeros((3, 1))

    mag_A = np.zeros((3, 3))
    mag_b = np.zeros((3, 1))


    # How to reconcile with differently spaced data:
    # Join the pandas dataframe
    # Keep the extra timesteps
    # If the timestep difference from sensor to sensor is < epsilon, then combine them into one row

    for i in time_step:
        # Whatever sensor data is in that time step
        # Run that throiugh the ekf


    # Output the EKF logged state to a csv file for viz, etc. later
    # Print out the XTE, ATE and other metrics

    pass



