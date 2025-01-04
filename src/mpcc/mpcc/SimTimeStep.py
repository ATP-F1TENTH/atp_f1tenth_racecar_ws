
import numpy as np
from scipy.integrate import solve_ivp


class SimTimeStep(None):
    """
    This class contains functions to calculate and simulate the dynamics of a vehicle
    modeled as a bicycle for one time step using the provided state, input, and
    model parameters, returning the state at the next time step.
    """

    def __init__(self) -> None:
        pass


    def _fx_bicycle(self, t, x, u, model_params):
        """
        Calculates the derivatives of the state for the bicycle model.
        Args:
            t (float): Current time (unused but required for solve_ivp).
            x (array): Current state.
            u (array): Current input.
            model_params (dict): Model parameters.
        Returns:
            xdot (array): State derivatives.
        """
        Cm1 = model_params['Cm1']
        Cm2 = model_params['Cm2']
        Cr0 = model_params['Cr0']
        Cr2 = model_params['Cr2']
        
        B_r = model_params['Br']
        C_r = model_params['Cr']
        D_r = model_params['Dr']
        B_f = model_params['Bf']
        C_f = model_params['Cf']
        D_f = model_params['Df']
        
        m = model_params['m']
        Iz = model_params['Iz']
        l_f = model_params['lf']
        l_r = model_params['lr']

        phi   = x[2]
        v_x   = x[3]
        v_y   = x[4]
        omega = x[5]
        D     = u[0]
        delta = u[1]
        
        alpha_f = -np.arctan2(l_f * omega + v_y, np.abs(v_x)) + delta
        alpha_r = np.arctan2(l_r * omega - v_y, np.abs(v_x))

        F_fy = D_f * np.sin(C_f * np.arctan(B_f * alpha_f))
        F_ry = D_r * np.sin(C_r * np.arctan(B_r * alpha_r))

        F_rx = (Cm1 * D - Cm2 * D * v_x - Cr0 - Cr2 * v_x ** 2)

        xdot = np.array([
            v_x * np.cos(phi) - v_y * np.sin(phi),
            v_y * np.cos(phi) + v_x * np.sin(phi),
            omega,
            1/m * (F_rx - F_fy * np.sin(delta) + m * v_y * omega),
            1/m * (F_ry + F_fy * np.cos(delta) - m * v_x * omega),
            1/Iz * (F_fy * l_f * np.cos(delta) - F_ry * l_r),
            u[2]
        ])
        
        return xdot

    
    
    def sim_time_step(self, x, u, Ts, model_params):
        """
        Simulates the dynamics of a vehicle modeled as a bicycle for one time step.
        Args:
            x (array): Current state.
            u (array): Current input.
            Ts (float): Sampling time.
            model_params (dict): Model parameters.
        Returns:
            xp (array): State at the next time step.
        """
        
        # x state
        # u input
        # Ts sampling time
        x0 = x
        
        # Use solve_ivp to replace ode45
        sol = solve_ivp(lambda t, x: self._fx_bicycle(t, x, u, model_params), [0, Ts], x0, t_eval=[Ts])
        xp = sol.y[:, -1]  # Last value of the solution
        return xp