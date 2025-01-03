import numpy as np

class updating_phi(None):
    """
    The class unwrape X0 so that the angle x0(2) remains within the range (−π,π] by adjusting 
    its value if it exceeds this range.
    """

    def __init__(self) -> None:
        pass

    def unWrapX0(x0):
        if x0[2] > np.pi:
            x0[2] -= 2 * np.pi
        if x0[2] <= -np.pi:
            x0[2] += 2 * np.pi
        return x0