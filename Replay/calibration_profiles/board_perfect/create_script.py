import pandas as pd
import numpy as np
import os

sensors = ["gyro", "accel", "mag"]

path = os.path.join("Replay", "calibration_profiles", "board_perfect")

for sensor in sensors:
    A_np = np.eye(3)
    A = pd.DataFrame(A_np, columns=["x", "y", "z"])

    A.to_csv(os.path.join(path, f"{sensor}_A.csv"), index=False)

    b_np = np.zeros((1, 3))
    b = pd.DataFrame(b_np, columns=["x", "y", "z"])
    b.to_csv(os.path.join(path, f"{sensor}_b.csv"), index=False)

# TODO change it to save to current board_perfect directory