import os
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as R


# Dear Chat our lord and savior, thanks for this gravious plotting code I cannot be bothered writing


# ============================================================
# Generic plotting utilities
# ============================================================

def plot_comparison_page(
    df1,
    df2,
    plots,
    time_column=None,
    page_title=None,
    figsize=(12, 8),
    save_path=None,
    label1="Frame 1",
    label2="Frame 2"
):
    """
    Create a page containing multiple time-series plots comparing two dataframes.

    Parameters
    ----------
    df1 : pandas.DataFrame
        First dataset (e.g., Frame 1 / Baseline).

    df2 : pandas.DataFrame
        Second dataset (e.g., Frame 2 / Comparison).

    plots : list of dict
        Each dictionary defines one subplot. Example:

        {
            "title": "Quaternion W",
            "col": "q_w",          # Column name present in both df1 and df2
            "ylabel": "q_w"
        }
        Alternatively, if column names differ across files:
        {
            "title": "Quaternion W",
            "col1": "q_w_frame1",
            "col2": "q_w_frame2",
            "ylabel": "q_w"
        }

    time_column : str or None
        Column to use for the x-axis. If None, row index is used.

    page_title : str or None
        Overall figure title.

    figsize : tuple
        Figure size.

    save_path : str or None
        If provided, save the figure to this path.

    label1 : str
        Legend label for df1.

    label2 : str
        Legend label for df2.

    Returns
    -------
    fig, axes
        Matplotlib figure and axes.
    """

    n_plots = len(plots)

    fig, axes = plt.subplots(
        n_plots,
        1,
        figsize=figsize,
        sharex=True
    )

    # If there is only one subplot, matplotlib doesn't return a list
    if n_plots == 1:
        axes = [axes]

    if time_column is not None:
        x1 = df1[time_column]
        x2 = df2[time_column]
        x_label = time_column
    else:
        x1 = np.arange(len(df1))
        x2 = np.arange(len(df2))
        x_label = "Sample"

    for ax, plot in zip(axes, plots):
        col1 = plot.get("col1", plot.get("col"))
        col2 = plot.get("col2", plot.get("col"))

        ax.plot(
            x1,
            df1[col1],
            label=label1,
            linewidth=1.5
        )

        ax.plot(
            x2,
            df2[col2],
            label=label2,
            linewidth=1.5
        )

        ax.set_title(plot["title"])
        ax.set_ylabel(plot.get("ylabel", ""))
        ax.grid(True, alpha=0.3)
        ax.legend()

    axes[-1].set_xlabel(x_label)

    if page_title is not None:
        fig.suptitle(page_title, fontsize=16)

    fig.tight_layout()

    # Make room for the overall title
    if page_title is not None:
        fig.subplots_adjust(top=0.93)

    if save_path is not None:
        fig.savefig(
            save_path,
            dpi=300,
            bbox_inches="tight"
        )

    return fig, axes


# ============================================================
# Quaternion utilities
# ============================================================

def quaternion_error_angle(df1, df2):
    """
    Calculate the angular error between quaternions in two dataframes
    using scipy Rotation.

    CSV convention assumed:
        [q_x, q_y, q_z, q_w]

    Returns
    -------
    error_deg : ndarray
        Quaternion attitude error in degrees.
    """

    q1 = df1[["q_x", "q_y", "q_z", "q_w"]].to_numpy()
    q2 = df2[["q_x", "q_y", "q_z", "q_w"]].to_numpy()

    # Convert quaternions to scipy Rotation objects
    R1 = R.from_quat(q1, scalar_first=False)
    R2 = R.from_quat(q2, scalar_first=False)

    # Rotation that takes frame 2 -> frame 1
    R_error = R1 * R2.inv()

    # Rotation-vector magnitude = attitude error angle
    error_rad = np.linalg.norm(
        R_error.as_rotvec(),
        axis=1
    )

    return np.degrees(error_rad)


# ============================================================
# Main CSV plotting function
# ============================================================

def plot_navigation_results(
    csv_path1,
    csv_path2,
    time_column=None,
    save_dir=None,
    label1="Frame 1",
    label2="Frame 2"
):
    """
    Load navigation results from two CSVs and generate comparative plots:

        Page 1:
            Quaternion W
            Quaternion X
            Quaternion Y
            Quaternion Z

        Page 2:
            NED X
            NED Y
            NED Z

        Page 3:
            Quaternion attitude error between Frame 1 and Frame 2

    Parameters
    ----------
    csv_path1 : str
        Path to first CSV.

    csv_path2 : str
        Path to second CSV.

    time_column : str or None
        Time column in the CSVs. If None, sample number is used.

    save_dir : str or None
        Directory where plots should be saved.
        If None, plots are only displayed.

    label1 : str
        Display label for the first dataset.

    label2 : str
        Display label for the second dataset.
    """

    df1 = pd.read_csv(csv_path1)
    df2 = pd.read_csv(csv_path2)

    # --------------------------------------------------------
    # Quaternion page
    # --------------------------------------------------------

    quaternion_plots = [
        {
            "title": "Quaternion W",
            "col": "q_w",
            "ylabel": "q_w"
        },
        {
            "title": "Quaternion X",
            "col": "q_x",
            "ylabel": "q_x"
        },
        {
            "title": "Quaternion Y",
            "col": "q_y",
            "ylabel": "q_y"
        },
        {
            "title": "Quaternion Z",
            "col": "q_z",
            "ylabel": "q_z"
        }
    ]

    quat_save_path = None
    if save_dir is not None:
        os.makedirs(save_dir, exist_ok=True)
        quat_save_path = f"{save_dir}/quaternion_comparison.png"

    plot_comparison_page(
        df1=df1,
        df2=df2,
        plots=quaternion_plots,
        time_column=time_column,
        page_title=f"Quaternion Comparison ({label1} vs {label2})",
        save_path=quat_save_path,
        label1=label1,
        label2=label2
    )

    # --------------------------------------------------------
    # Position page
    # --------------------------------------------------------

    position_plots = [
        {
            "title": "NED X Position",
            "col": "ned_x",
            "ylabel": "North (m)"
        },
        {
            "title": "NED Y Position",
            "col": "ned_y",
            "ylabel": "East (m)"
        },
        {
            "title": "NED Z Position",
            "col": "ned_z",
            "ylabel": "Down (m)"
        }
    ]

    position_save_path = None
    if save_dir is not None:
        position_save_path = f"{save_dir}/position_comparison.png"

    plot_comparison_page(
        df1=df1,
        df2=df2,
        plots=position_plots,
        time_column=time_column,
        page_title=f"NED Position Comparison ({label1} vs {label2})",
        save_path=position_save_path,
        label1=label1,
        label2=label2
    )

    # --------------------------------------------------------
    # Quaternion attitude error
    # --------------------------------------------------------

    quat_error_deg = quaternion_error_angle(df1, df2)

    if time_column is not None:
        x = df1[time_column]
        x_label = time_column
    else:
        x = np.arange(len(df1))
        x_label = "Sample"

    fig, ax = plt.subplots(figsize=(12, 4))

    ax.plot(
        x,
        quat_error_deg,
        linewidth=1.5,
        color="crimson"
    )

    ax.set_title(f"Quaternion Attitude Error ({label1} vs {label2})")
    ax.set_xlabel(x_label)
    ax.set_ylabel("Error (deg)")
    ax.grid(True, alpha=0.3)

    fig.tight_layout()

    if save_dir is not None:
        fig.savefig(
            f"{save_dir}/quaternion_error.png",
            dpi=300,
            bbox_inches="tight"
        )


if __name__ == "__main__":

    plot_navigation_results(
        csv_path1=os.path.join("Replay", "runs", "simulations", "run_0", "inertial_estimator_log.csv"),
        csv_path2=os.path.join("Replay", "runs", "simulations", "run_0", "perfect_estimator_log.csv"),
        time_column="time",
        save_dir=os.path.join("Replay", "runs", "simulations", "comparison_plots"),
        label1="Inertial",
        label2="Perfect"
    )

    plt.show()