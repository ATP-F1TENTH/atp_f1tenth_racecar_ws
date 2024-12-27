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
    #Xbar_k are current values - think of "working point", e.g. like in taylor series
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



    # Derivatives of the force laws
    dFrx_dvx = -Cm2 * D - 2 * Cr2 * v_x
    dFrx_dD = Cm1 - Cm2 * v_x

    dFry_dvx = ((B_r * C_r * D_r * np.cos(C_r * np.arctan(B_r * alpha_r))) / (1 + B_r**2 * alpha_r**2)) * \
            (-(l_r * omega - v_y) / ((-l_r * omega + v_y)**2 + v_x**2))

    dFry_dvy = ((B_r * C_r * D_r * np.cos(C_r * np.arctan(B_r * alpha_r))) / (1 + B_r**2 * alpha_r**2)) * \
            ((-v_x) / ((-l_r * omega + v_y)**2 + v_x**2))

    dFry_domega = ((B_r * C_r * D_r * np.cos(C_r * np.arctan(B_r * alpha_r))) / (1 + B_r**2 * alpha_r**2)) * \
                ((l_r * v_x) / ((-l_r * omega + v_y)**2 + v_x**2))

    dFfy_dvx = ((B_f * C_f * D_f * np.cos(C_f * np.arctan(B_f * alpha_f))) / (1 + B_f**2 * alpha_f**2)) * \
            ((l_f * omega + v_y) / ((l_f * omega + v_y)**2 + v_x**2))

    dFfy_dvy = ((B_f * C_f * D_f * np.cos(C_f * np.arctan(B_f * alpha_f))) / (1 + B_f**2 * alpha_f**2)) * \
            (-v_x / ((l_f * omega + v_y)**2 + v_x**2))

    dFfy_domega = ((B_f * C_f * D_f * np.cos(C_f * np.arctan(B_f * alpha_f))) / (1 + B_f**2 * alpha_f**2)) * \
                ((-l_f * v_x) / ((l_f * omega + v_y)**2 + v_x**2))

    dFfy_ddelta = (B_f * C_f * D_f * np.cos(C_f * np.arctan(B_f * alpha_f))) / (1 + B_f**2 * alpha_f**2)

    # Jacobians of the system
    df1_dphi = -v_x * np.sin(phi) - v_y * np.cos(phi)
    df1_dvx = np.cos(phi)
    df1_dvy = -np.sin(phi)

    df2_dphi = -v_y * np.sin(phi) + v_x * np.cos(phi)
    df2_dvx = np.sin(phi)
    df2_dvy = np.cos(phi)

    df3_domega = 1

    df4_dvx = 1 / m * (dFrx_dvx - dFfy_dvx * np.sin(delta))
    df4_dvy = 1 / m * (-dFfy_dvy * np.sin(delta) + m * omega)
    df4_domega = 1 / m * (-dFfy_domega * np.sin(delta) + m * v_y)
    df4_dD = 1 / m * dFrx_dD
    df4_ddelta = 1 / m * (-dFfy_ddelta * np.sin(delta) - F_fy * np.cos(delta))

    df5_dvx = 1 / m * (dFry_dvx + dFfy_dvx * np.cos(delta) - m * omega)
    df5_dvy = 1 / m * (dFry_dvy + dFfy_dvy * np.cos(delta))
    df5_domega = 1 / m * (dFry_domega + dFfy_domega * np.cos(delta) - m * v_x)
    df5_ddelta = 1 / m * (dFfy_ddelta * np.cos(delta) - F_fy * np.sin(delta))

    df6_dvx = 1 / Iz * (dFfy_dvx * l_f * np.cos(delta) - dFry_dvx * l_r)
    df6_dvy = 1 / Iz * (dFfy_dvy * l_f * np.cos(delta) - dFry_dvy * l_r)
    df6_domega = 1 / Iz * (dFfy_domega * l_f * np.cos(delta) - dFry_domega * l_r)
    df6_ddelta = 1 / Iz * (dFfy_ddelta * l_f * np.cos(delta) - F_fy * l_f * np.sin(delta))

    # Construct the Jacobian matrices
    Ac = np.array([
        [0, 0, df1_dphi, df1_dvx, df1_dvy, 0],
        [0, 0, df2_dphi, df2_dvx, df2_dvy, 0],
        [0, 0, 0, 0, 0, df3_domega],
        [0, 0, 0, df4_dvx, df4_dvy, df4_domega],
        [0, 0, 0, df5_dvx, df5_dvy, df5_domega],
        [0, 0, 0, df6_dvx, df6_dvy, df6_domega]
    ])

    Bc = np.array([
        [0, 0],
        [0, 0],
        [0, 0],
        [df4_dD, df4_ddelta],
        [0, df5_ddelta],
        [0, df6_ddelta]
    ])

    gc = f - Ac @ Xbar_k[:sx] - Bc @ Ubar_k[:su]

    # Augment Bc for discretization
    #! check this line if correct
    #! gc.reshape to transpose the matrix?
    Bc_aug = np.hstack((Bc, gc.reshape(-1, 1)))

    # Discretization using matrix exponential
    #! discrete calculation of matrices, tmp as result
    #! Approach: x_k+1 = x_k + A * Δt * x_k
    tmp = expm(np.block([[Ac, Bc_aug],[np.zeros((su + 1, sx + su + 1))]]) * Ts)

    Ad = np.zeros((sx + 1, sx + 1))
    Bd = np.zeros((sx + 1, su + 1))
    gd = np.zeros((sx + 1))

    Ad[:sx, :sx] = tmp[:sx, :sx]
    Bd[:sx, :su] = tmp[:sx, sx:sx + su]
    gd[:sx] = tmp[:sx, sx + su]

    # Avoid numerical errors
    #! check this line if correct - still some doubt
    Ad[-1, -1] = 1
    Bd[-1, -1] = Ts

    return Ad, Bd, gd

