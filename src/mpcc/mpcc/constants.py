
from numpy import diag, array, pi, eye


class MatCons():
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
        self.__mat_cons["Tx"] = diag(1.0 / array([3, 3, 2 * pi, 4, 2, 7, 30]))
        self.__mat_cons["Tu"] = diag(1.0 / array([1, 0.35, 6]))

        self.__mat_cons["invTx"] = diag([3, 3, 2 * pi, 4, 2, 7, 30])
        self.__mat_cons["invTu"] = diag([1, 0.35, 6])

        self.__mat_cons["TDu"] = eye(3)
        self.__mat_cons["invTDu"] = eye(3)

        # State-input bounds (normalized)
        self.__mat_cons["bounds"] = array(
            [   [-1, -1, -3, 0, -1, -1, 0, -0.1, -1, 0, -1, -1, -5],
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
    


class CarConstants(object):
    """
    Entity class defining all constant model parameter in a dictionary
    """
    
    def __new__(self):
        
        self.__model_params = {

            "sx": 7,  # number of states
            "su": 3,  # number of inputs
            "nx": 7,  # number of states
            "nu": 3,  # number of inputs

            # State indices
            "stateindex_x": 1,  # x position
            "stateindex_y": 2,  # y position
            "stateindex_phi": 3,  # orientation
            "stateindex_vx": 4,  # longitudinal velocity
            "stateindex_vy": 5,  # lateral velocity
            "stateindex_omega": 6,  # yaw rate
            "stateindex_theta": 7,  # virtual position

            # Input indices
            "inputindex_D": 1,  # duty cycle
            "inputindex_delta": 2,  # steering angle
            "inputindex_vtheta": 3,  # virtual speed

            # Physical parameters
            "m": 4.675,         #source Ilja
            "Iz": 0.1751,       #source Ilja
            "lf": 0.162,        #source Ilja
            "lr": 0.162,        #source Ilja

            # Coefficients
            "Cm1": 0.5,         #source Presentation (Leah & Soeren)
            "Cm2": 0.1,         #source Presentation (Leah & Soeren)
            "Cr0": 1,           #yet to be defined - therefore initial value of 1
            "Cr2": 1,           #yet to be defined - therefore initial value of 1

            # Tire model parameters
            "Br": 8,            #source Presentation (Leah & Soeren)
            "Cr": 1.2,          #source Presentation (Leah & Soeren)
            "Dr": 0.8,          #source Presentation (Leah & Soeren)
            "Bf": 10,           #source Presentation (Leah & Soeren)  
            "Cf": 1.5,          #source Presentation (Leah & Soeren)
            "Df": 1,            #source Presentation (Leah & Soeren)

            # Dimensions
            "L": 0.6,           #source Ilja
            "W": 0.274,         #source Ilja
        }

        return self.__model_params