class linearizedModel():
    def __init__(self):
        pass

import numpy as np
from scipy.linalg import expm

#equations copied from liniger ( DiscretizedLinearizedModel )
#basically bicycle model

def discretized_linearized_model(Xbar_k, Ubar_k, model_params, Ts):
    """
    Returns the discretized, linearized model about (Xbar_k, Ubar_k)
    s.t. x(k+1) = Ad * x(k) + Bd * u(k) + gd

    Parameters:
        Xbar_k : np.array
            State vector at time k.
        Ubar_k : np.array
            Input vector at time k.
        model_params : dict
            Model parameters.
        Ts : float
            Sampling time.

    Returns:
        Ad : np.ndarray
            Discretized state transition matrix.
        Bd : np.ndarray
            Discretized input matrix.
        gd : np.ndarray
            Discretized offset vector.
    """

    
    sx = model_params["sx"] - 1     #sx = 7 -> therefore -1, bcs we only look at the 6 states from bicyle model
    su = model_params["su"] - 1     #sx = 7 -> therefore -1, bcs we only look at the 6 states from bicyle model

    # Extract model parameters
    Cm1 = model_params["Cm1"]
    Cm2 = model_params["Cm2"]
    Cr0 = model_params["Cr0"]
    Cr2 = model_params["Cr2"]
    B_r = model_params["Br"]
    C_r = model_params["Cr"]
    D_r = model_params["Dr"]
    B_f = model_params["Bf"]
    C_f = model_params["Cf"]
    D_f = model_params["Df"]
    m = model_params["m"]
    Iz = model_params["Iz"]
    l_f = model_params["lf"]
    l_r = model_params["lr"]

    # Extract states and inputs
    phi = Xbar_k[model_params["stateindex_phi"]]
    v_x = Xbar_k[model_params["stateindex_vx"]]
    v_y = Xbar_k[model_params["stateindex_vy"]]
    omega = Xbar_k[model_params["stateindex_omega"]]

    D = Ubar_k[model_params["inputindex_D"]]
    delta = Ubar_k[model_params["inputindex_delta"]]
    vtheta = Ubar_k[model_params["inputindex_vtheta"]]

    # Handle low velocities
    if v_x < 0.5:
        v_y, omega, delta = 0, 0, 0
        v_x = max(v_x, 0.3)

    # Tire slip angles
    alpha_f = -np.arctan2(l_f * omega + v_y, v_x) + delta
    alpha_r = np.arctan2(l_r * omega - v_y, v_x)

    # Tire forces
    F_fy = D_f * np.sin(C_f * np.arctan(B_f * alpha_f))
    F_ry = D_r * np.sin(C_r * np.arctan(B_r * alpha_r))
    F_rx = Cm1 * D - Cm2 * D * v_x - Cr0 - Cr2 * v_x**2

    # Dynamics equations
    f = np.array([
        v_x * np.cos(phi) - v_y * np.sin(phi),
        v_y * np.cos(phi) + v_x * np.sin(phi),
        omega,
        (1 / m) * (F_rx - F_fy * np.sin(delta) + m * v_y * omega),
        (1 / m) * (F_ry + F_fy * np.cos(delta) - m * v_x * omega),
        (1 / Iz) * (F_fy * l_f * np.cos(delta) - F_ry * l_r)
    ])

    # Linearization: Jacobians (partial derivatives)
    Ac = np.zeros((sx, sx))  # State transition matrix
    Bc = np.zeros((sx, su))  # Control matrix
    gc = f - Ac @ Xbar_k[:sx] - Bc @ Ubar_k[:su]

    # Augment Bc for discretization
    Bc_aug = np.hstack((Bc, gc.reshape(-1, 1)))

    # Discretization using matrix exponential
    tmp = expm(np.block([
        [Ac, Bc_aug],
        [np.zeros((su + 1, sx + su + 1))]
    ]) * Ts)

    Ad = np.zeros((sx + 1, sx + 1))
    Bd = np.zeros((sx + 1, su + 1))
    gd = np.zeros((sx + 1))

    Ad[:sx, :sx] = tmp[:sx, :sx]
    Bd[:sx, :su] = tmp[:sx, sx:sx + su]
    gd[:sx] = tmp[:sx, sx + su]

    # Avoid numerical errors
    Ad[-1, -1] = 1
    Bd[-1, -1] = Ts

    return Ad, Bd, gd

