class MPC_Mat_Cons():
    def __init__(self):
        pass

#generates and return a dictionary of parameters and settings
#values copied from liniger ( getMPC_vars )

import numpy as np

def get_mat_cons():
    mat_cons = {}

    # MPC settings
    mat_cons["N"] = 40  # prediction horizon
    mat_cons["Ts"] = 0.02  # sampling time
    mat_cons["ModelNo"] = 1  # used model
    mat_cons["fullBound"] = True  # use bounds on all opt variables

    # State-input scaling
    mat_cons["Tx"] = np.diag(1.0 / np.array([3, 3, 2 * np.pi, 4, 2, 7, 30]))
    mat_cons["Tu"] = np.diag(1.0 / np.array([1, 0.35, 6]))

    mat_cons["invTx"] = np.diag([3, 3, 2 * np.pi, 4, 2, 7, 30])
    mat_cons["invTu"] = np.diag([1, 0.35, 6])

    mat_cons["TDu"] = np.eye(3)
    mat_cons["invTDu"] = np.eye(3)

    # State-input bounds (normalized)
    mat_cons["bounds"] = np.array([
        [-1, -1, -3, 0, -1, -1, 0, -0.1, -1, 0, -1, -1, -5],
        [1, 1, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 5]
    ]).T

    # Cost parameters
    mat_cons["qC"] = 0.1                # contouring cost
    mat_cons["qCNmult"] = 10            # terminal contouring cost multiplier
    mat_cons["qL"] = 1000               # lag cost
    mat_cons["qVtheta"] = 0.02          # theta maximization cost
    mat_cons["qOmega"] = 1e-5           # yaw rate regularization cost
    mat_cons["qOmegaNmult"] = 10        # terminal yaw rate regularization cost multiplier

    mat_cons["rD"] = 1e-4               # cost on duty cycle
    mat_cons["rDelta"] = 1e-4           # cost on steering
    mat_cons["rVtheta"] = 1e-4          # cost on virtual velocity

    mat_cons["rdD"] = 0.01              # cost on change of duty cycle
    mat_cons["rdDelta"] = 1             # cost on change of steering
    mat_cons["rdVtheta"] = 0.001        # cost on change of virtual velocity

    mat_cons["q_eta"] = 250             # cost on soft constraints
    mat_cons["costScale"] = 1           # scaling of the cost for better numerics

    return mat_cons

