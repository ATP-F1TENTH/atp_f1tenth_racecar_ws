
import numpy as np


class MpcMatCons():
    """
    Entity class to store mpc mat constants
        -> Generate and return a dictionary of parameters and settings
        => values copied from liniger (getMPC_vars) 
    """

    def __new__(self):
        
        self.__mat_cons = {}

        # MPC settings
        self.__mat_cons["N"] = 40  # prediction horizon
        self.__mat_cons["Ts"] = 0.02  # sampling time
        self.__mat_cons["ModelNo"] = 1  # used model
        self.__mat_cons["fullBound"] = True  # use bounds on all opt variables

        # State-input scaling
        self.__mat_cons["Tx"] = np.diag(1.0 / np.array([3, 3, 2 * np.pi, 4, 2, 7, 30]))
        self.__mat_cons["Tu"] = np.diag(1.0 / np.array([1, 0.35, 6]))

        self.__mat_cons["invTx"] = np.diag([3, 3, 2 * np.pi, 4, 2, 7, 30])
        self.__mat_cons["invTu"] = np.diag([1, 0.35, 6])

        self.__mat_cons["TDu"] = np.eye(3)
        self.__mat_cons["invTDu"] = np.eye(3)

        # State-input bounds (normalized)
        self.__mat_cons["bounds"] = np.array([
            [-1, -1, -3, 0, -1, -1, 0, -0.1, -1, 0, -1, -1, -5],
            [1, 1, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 5]
        ]).T

        # Cost parameters
        self.__mat_cons["qC"] = 0.1                # contouring cost
        self.__mat_cons["qCNmult"] = 10            # terminal contouring cost multiplier
        self.__mat_cons["qL"] = 1000               # lag cost
        self.__mat_cons["qVtheta"] = 0.02          # theta maximization cost
        self.__mat_cons["qOmega"] = 1e-5           # yaw rate regularization cost
        self.__mat_cons["qOmegaNmult"] = 10        # terminal yaw rate regularization cost multiplier

        self.__mat_cons["rD"] = 1e-4               # cost on duty cycle
        self.__mat_cons["rDelta"] = 1e-4           # cost on steering
        self.__mat_cons["rVtheta"] = 1e-4          # cost on virtual velocity

        self.__mat_cons["rdD"] = 0.01              # cost on change of duty cycle
        self.__mat_cons["rdDelta"] = 1             # cost on change of steering
        self.__mat_cons["rdVtheta"] = 0.001        # cost on change of virtual velocity

        self.__mat_cons["q_eta"] = 250             # cost on soft constraints
        self.__mat_cons["costScale"] = 1           # scaling of the cost for better numerics

        return self.__mat_cons