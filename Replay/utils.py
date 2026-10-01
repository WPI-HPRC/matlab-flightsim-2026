import numpy as np
from scipy.spatial.transform import Rotation as R
import pymap3d as pm

def dcm_ecef2ned(lla):
    lat = lla[0]
    lon = lla[1]

    # R_ET = [
    #     -sind(lat)*cosd(lon), -sind(lon), -cosd(lat)*cosd(lon);
    #     -sind(lat)*sind(lon), cosd(lon), -cosd(lat)*sind(lon);
    #     cosd(lat), 0, -sind(lat);
    # ];

    dcm_ecef2ned = R.from_matrix(np.array([
        [-np.sin(np.radians(lat)) * np.cos(np.radians(lon)), -np.sin(np.radians(lon)), -np.cos(np.radians(lat)) * np.cos(np.radians(lon))],
        [-np.sin(np.radians(lat)) * np.sin(np.radians(lon)), np.cos(np.radians(lon)), -np.cos(np.radians(lat)) * np.sin(np.radians(lon))],
        [np.cos(np.radians(lat)), 0, -np.sin(np.radians(lat))]
    ]))

    return dcm_ecef2ned.inv()

def lla2ecef(lla):
    x, y, z = pm.geodetic2ecef(lla[0], lla[1], lla[2])
    pos_ecef = np.array([x, y, z])
    return pos_ecef

def ecef2lla(ecef):
    lat, lon, alt = pm.ecef2geodetic(ecef[0], ecef[1], ecef[2])
    return np.array([lat, lon, alt])

import numpy as np
from scipy.spatial.transform import Rotation


# Dear chat thanks for this code. I've written this a couple times, but want to ensure no bugs
def TRIAD(a_b, a_i, m_b, m_i):
    """
    TRIAD attitude determination.

    Parameters
    ----------
    a_b : array_like, shape (3,)
        First vector measured in the body frame.
    a_i : array_like, shape (3,)
        First reference vector in the inertial frame.

    m_b : array_like, shape (3,)
        Second vector measured in the body frame.
    m_i : array_like, shape (3,)
        Second reference vector in the inertial frame.

    Returns
    -------
    Rotation
        scipy Rotation object representing the rotation BODY -> INERTIAL.
    """

    # Make writable copies
    a_b = np.array(a_b, dtype=float, copy=True)
    a_i = np.array(a_i, dtype=float, copy=True)
    m_b = np.array(m_b, dtype=float, copy=True)
    m_i = np.array(m_i, dtype=float, copy=True)

    # Normalize input vectors
    a_b /= np.linalg.norm(a_b)
    a_i /= np.linalg.norm(a_i)
    m_b /= np.linalg.norm(m_b)
    m_i /= np.linalg.norm(m_i)

    # -------------------------
    # Construct body triad
    # -------------------------

    t1_b = a_b
    t2_b = np.cross(a_b, m_b)
    t2_b /= np.linalg.norm(t2_b)
    t3_b = np.cross(t1_b, t2_b)

    # -------------------------
    # Construct inertial triad
    # -------------------------

    t1_i = a_i
    t2_i = np.cross(a_i, m_i)
    t2_i /= np.linalg.norm(t2_i)
    t3_i = np.cross(t1_i, t2_i)

    # Triad matrices: columns are basis vectors
    T_b = np.column_stack((t1_b, t2_b, t3_b))
    T_i = np.column_stack((t1_i, t2_i, t3_i))

    # Body -> inertial
    C_ib = T_i @ T_b.T

    # Convert DCM to scipy Rotation
    return Rotation.from_matrix(C_ib)