import pandas as pd
import numpy as np
import os


path = os.path.join("Replay", "calibration_profiles", "board_perfect")
gyro_data_path = os.path.join(path, "gyro_test_data.csv")
gyro_data = pd.read_csv(gyro_data_path)

start_ts = 2
end_ts = 8

gyro_data_subset = gyro_data[(gyro_data['time'] >= start_ts) & (gyro_data['time'] <= end_ts)]

bias = gyro_data_subset[['x', 'y', 'z']].mean().to_numpy()
std = gyro_data_subset[['x', 'y', 'z']].std().to_numpy()
print(f"Gyro bias: {bias}. Gyro std: {std}")

A_np = np.eye(3)
A = pd.DataFrame(A_np, columns=["x", "y", "z"])
A.to_csv(os.path.join(path, "gyro_A.csv"), index=False)
b_np = std
b = pd.DataFrame(b_np.reshape(1, 3), columns=["x", "y", "z"])
b.to_csv(os.path.join(path, "gyro_b.csv"), index=False)
