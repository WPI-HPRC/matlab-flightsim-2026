import pandas as pd
import numpy as np
import os

# Purpose: To create a dummy version of what the calibration CSV files look like

sensors = ["gyro", "accel", "mag"]

path = os.path.join("Replay", "calibration_profiles", "board_perfect")

num_points = 10000

for sensor in sensors:
    A_np = np.eye(3)
    A = pd.DataFrame(A_np, columns=["x", "y", "z"])

    A.to_csv(os.path.join(path, f"{sensor}_A.csv"), index=False)

    b_np = np.zeros((1, 3))
    b = pd.DataFrame(b_np, columns=["x", "y", "z"])
    b.to_csv(os.path.join(path, f"{sensor}_b.csv"), index=False)


    test_data = []

    bias_std = 0.1
    wn_std = 0.01

    bias = bias_std * np.random.randn(3)

    for point in range(num_points): # Made into a loop so that can eventually add time variant noise
        wn = wn_std * np.random.randn(3)
        test_data.append(bias + wn)

    df = pd.DataFrame(test_data, columns=["x", "y", "z"])
    df["time"] = np.linspace(0, 10, num_points) # some dummy time

    df.to_csv(os.path.join(path, f"{sensor}_test_data.csv"), index=False)

        

# TODO change it to save to current board_perfect directory