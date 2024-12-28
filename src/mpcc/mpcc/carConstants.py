

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