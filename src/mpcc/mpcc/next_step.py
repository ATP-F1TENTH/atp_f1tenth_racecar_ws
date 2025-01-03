import numpy as np

class next_step(None):
    """
    This function updates and augments state and control trajectories for the next iteration
    of an MPC optimization, ensuring continuity for angular states and simulating the next state based on the current inputs.
    """

    def __init__(self) -> None:
        pass

    def aug_state(x, u, x0, MPC_vars, ModelParams, tl):

        nx = ModelParams['nx']
        nu = ModelParams['nu']
        N = MPC_vars['N']
        Ts = MPC_vars['Ts']
        index_phi = ModelParams['stateindex_phi']
        index_theta = ModelParams['stateindex_theta']

        xTemp = np.zeros((nx, N + 1))
        uTemp = np.zeros((nu, N))

        xTemp[:, 0] = x0
        uTemp[:, 0] = u[:, 1]

        for j in range(1, N - 1):   
            xTemp[:, j] = x[:, j + 1]
            uTemp[:, j] = u[:, j + 1]

        j = N
        xTemp[:, j] = x[:, j + 1]
        uTemp[:, j] = u[:, j]

        j = N+1
        xTemp[:, j] = SimTimeStep(x[:, N+1], u[:, N], Ts, ModelParams)

        if xTemp[index_phi, 0] - xTemp[index_phi, 1] > np.pi:
            xTemp[index_phi, 1:] += 2 * np.pi
        if xTemp[index_phi, 0] - xTemp[index_phi, 1] < -np.pi:
            xTemp[index_phi, 1:] -= 2 * np.pi

        if xTemp[index_theta, 0] - xTemp[index_theta, 1] < -0.75 * tl:
            xTemp[index_theta, 1:] -= tl

        return xTemp, uTemp
