
import os
import sys
import numpy as np
import pandas as pd

if __name__ == "__main__":
    # Load in csv files

    # Iterate over timesteps and calculate ekf output


    gps_data = None

    max_height = gps_data['altitude'].max()

    
    max_idx = gps_data['altitude'].idxmax()

    ned_pos_max = ekf.ned_pos[max_idx]




    err = ekf.ned_pos - gps_data.loc[max_idx, ["north", "east", "down"]].to_numpy()

    XTE = np.linalg.norm(err[:2]) # More attritable to gyro issues
    ATE = np.abs(err[2]) # More attritable to accelerometer/barometer issues





    pass