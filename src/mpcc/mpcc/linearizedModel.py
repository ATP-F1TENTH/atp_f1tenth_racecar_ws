
import numpy as np
from scipy.linalg import expm


class LinearizedModel(None):
    """
    Class creating the discretized, linearized model about (Xbar_k, Ubar_k)
        => s.t. x(k+1) = Ad * x(k) + Bd * u(k) + gd
        => equations copied from liniger ( DiscretizedLinearizedModel )
        => basically bicycle model
    """

    def __init__(self, Xbar_k, Ubar_k, model_params, Ts) -> None:
        """
        Parameters:
            Xbar_k : np.array
                State vector at time k.
            Ubar_k : np.array
                Input vector at time k.
            model_params : dict
                Model parameters.
            Ts : float
                Sampling time.
        """
        
        sx = model_params["sx"] - 1     #sx = 7 -> therefore -1, bcs we only look at the 6 states from bicyle model
        su = model_params["su"] - 1     #sx = 7 -> therefore -1, bcs we only look at the 6 states from bicyle model

        # Extract model parameters
        self._Cm1 = model_params["Cm1"]
        self._Cm2 = model_params["Cm2"]
        self._Cr0 = model_params["Cr0"]
        self._Cr2 = model_params["Cr2"]
        self._B_r = model_params["Br"]
        self._C_r = model_params["Cr"]
        self._D_r = model_params["Dr"]
        self._B_f = model_params["Bf"]
        self._C_f = model_params["Cf"]
        self._D_f = model_params["Df"]
        self._m = model_params["m"]
        self._Iz = model_params["Iz"]
        self._l_f = model_params["lf"]
        self._l_r = model_params["lr"]
        
        # Extract states and inputs
        # * Xbar_k are current values - think of "working point", e.g. like in taylor series
        self._phi = Xbar_k[model_params["stateindex_phi"]]
        tmp_vx = Xbar_k[model_params["stateindex_vx"]]
        self._v_x = tmp_vx if (tmp_vx >= 0.5) else max(tmp_vx, 0.3)                         # Handle low velocity
        self._v_y = Xbar_k[model_params["stateindex_vy"]] if (tmp_vx >= 0.5) else 0         # Handle low velocity
        self._omega = Xbar_k[model_params["stateindex_omega"]] if (tmp_vx >= 0.5) else 0    # Handle low velocity
        self._delta = Ubar_k[model_params["inputindex_delta"]] if (tmp_vx >= 0.5) else 0    # Handle low velocity
        self._D = Ubar_k[model_params["inputindex_D"]]
        self._vtheta = Ubar_k[model_params["inputindex_vtheta"]]

        # Tire parameter
        self._alpha_f, self._alpha_r = self._calculateTireSlipAngles()
        self._F_fy, self._F_ry, self._F_rx = self._calculateTireForces()

        # Dynamic equations
        self._f = self._getDynamicEquations()

        # Force Derivatives
        self._dFrx_dvx, self._dFrx_dD, self._dFry_dvx, self._dFry_dvy, self._dFry_domega, \
            self._dFfy_dvx, self._dFfy_dvy, self._dFfy_domega, self._dFfy_ddelta = self._getForceDerivatives()

        # Linearized model
        self.Ad, self.Bd, self.gd = self._discretized_linearized_model(Xbar_k, Ubar_k, sx, su, Ts)


    def getLinearizedModel(self) -> tuple:
        """
        Getter-Function for important model parameter in tuple
        Returns:
            Ad : np.ndarray
                Discretized state transition matrix.
            Bd : np.ndarray
                Discretized input matrix.
            gd : np.ndarray
                Discretized offset vector.
        """

        return (self.Ad, self.Bd, self.gd)


    def _calculateTireSlipAngles(self) -> tuple:
        alpha_f = -np.arctan2(self._l_f * self._omega + self._v_y, self._v_x) + self._delta
        alpha_r = np.arctan2(self._l_r * self._omega - self._v_y, self._v_x)
        return (alpha_f, alpha_r)
    

    def _calculateTireForces(self) -> tuple:
        F_fy = self._D_f * np.sin(self._C_f * np.arctan(self._B_f * self._alpha_f))
        F_ry = self._D_r * np.sin(self._C_r * np.arctan(self._B_r * self._alpha_r))
        F_rx = self._Cm1 * self._D - self._Cm2 * self._D * self._v_x - self._Cr0 - self._Cr2 * self._v_x**2
        return (F_fy, F_ry, F_rx)
    

    def _getDynamicEquations(self) -> np.array:
        f = np.array([
            self._v_x * np.cos(self._phi) - self._v_y * np.sin(self._phi),
            self._v_y * np.cos(self._phi) + self._v_x * np.sin(self._phi),
            self._omega,
            (1 / self._m)  * (self._F_rx - self._F_fy * np.sin(self._delta) + self._m * self._v_y * self._omega),
            (1 / self._m)  * (self._F_ry + self._F_fy * np.cos(self._delta) - self._m * self._v_x * self._omega),
            (1 / self._Iz) * (self._F_fy * self._l_f  * np.cos(self._delta) - self._F_ry * self._l_r)
        ])
        return f
    

    def _getForceDerivatives(self) -> tuple:

        dFrx_dvx = -self._Cm2 * self._D - 2 * self._Cr2 * self._v_x

        dFrx_dD = self._Cm1 - self._Cm2 * self._v_x

        # commen therm of dFry formulas
        tmp_dFry = ((self._B_r * self._C_r * self._D_r * np.cos(self._C_r * np.arctan(self._B_r * self._alpha_r))) / \
                    (1 + self._B_r**2 * self._alpha_r**2))

        dFry_dvx = tmp_dFry * (-(self._l_r * self._omega - self._v_y) / ((-self._l_r * self._omega + self._v_y)**2 + self._v_x**2))

        dFry_dvy = tmp_dFry * ((-self._v_x) / ((-self._l_r * self._omega + self._v_y)**2 + self._v_x**2))

        dFry_domega = tmp_dFry * ((self._l_r * self._v_x) / ((-self._l_r * self._omega + self._v_y)**2 + self._v_x**2))

        dFfy_ddelta = (self._B_f * self._C_f * self._D_f * np.cos(self._C_f * np.arctan(self._B_f * self._alpha_f))) / \
                      (1 + self._B_f**2 * self._alpha_f**2)

        dFfy_dvx = dFfy_ddelta * ((self._l_f * self._omega + self._v_y) / ((self._l_f * self._omega + self._v_y)**2 + self._v_x**2))

        dFfy_dvy = dFfy_ddelta * (-self._v_x / ((self._l_f * self._omega + self._v_y)**2 + self._v_x**2))

        dFfy_domega = dFfy_ddelta * ((-self._l_f * self._v_x) / ((self._l_f * self._omega + self._v_y)**2 + self._v_x**2))
        
        return (dFrx_dvx, dFrx_dD, dFry_dvx, dFry_dvy, dFry_domega, dFfy_dvx, dFfy_dvy, dFfy_domega, dFfy_ddelta)
    

    def _getFirstJacobians(self) -> tuple:

        df1_dphi = -self._v_x * np.sin(self._phi) - self._v_y * np.cos(self._phi)
        df1_dvx = np.cos(self._phi)
        df1_dvy = -np.sin(self._phi)

        return (df1_dphi, df1_dvx, df1_dvy)


    def _getSecondJacobians(self) -> tuple:
        
        df2_dphi = -self._v_y * np.sin(self._phi) + self._v_x * np.cos(self._phi)
        df2_dvx = np.sin(self._phi)
        df2_dvy = np.cos(self._phi)

        return (df2_dphi, df2_dvx, df2_dvy)


    def _getForthJacobians(self) -> tuple:

        factor = 1 / self._m

        df4_dvx = factor * (self._dFrx_dvx - self._dFfy_dvx * np.sin(self._delta))
        df4_dvy = factor * (-self._dFfy_dvy * np.sin(self._delta) + self._m * self._omega)
        df4_domega = factor * (-self._dFfy_domega * np.sin(self._delta) + self._m * self._v_y)
        df4_dD = factor * self._dFrx_dD
        df4_ddelta = factor * (-self._dFfy_ddelta * np.sin(self._delta) - self._F_fy * np.cos(self._delta))
        
        return (df4_dvx, df4_dvy, df4_domega, df4_dD, df4_ddelta)


    def _getFifthJacobians(self) -> tuple:

        factor = 1 / self._m

        df5_dvx = factor * (self._dFry_dvx + self._dFfy_dvx * np.cos(self._delta) - self._m * self._omega)
        df5_dvy = factor * (self._dFry_dvy + self._dFfy_dvy * np.cos(self._delta))
        df5_domega = factor * (self._dFry_domega + self._dFfy_domega * np.cos(self._delta) - self._m * self._v_x)
        df5_ddelta = factor * (self._dFfy_ddelta * np.cos(self._delta) - self._F_fy * np.sin(self._delta))

        return (df5_dvx, df5_dvy, df5_domega, df5_ddelta)


    def _getSixthJacobians(self) -> tuple:

        factor = 1 / self._Iz

        df6_dvx = factor * (self._dFfy_dvx * self._l_f * np.cos(self._delta) - self._dFry_dvx * self._l_r)
        df6_dvy = factor * (self._dFfy_dvy * self._l_f * np.cos(self._delta) - self._dFry_dvy * self._l_r)
        df6_domega = factor * (self._dFfy_domega * self._l_f * np.cos(self._delta) - self._dFry_domega * self._l_r)
        df6_ddelta = factor * (self._dFfy_ddelta * self._l_f * np.cos(self._delta) - self._F_fy * self._l_f * np.sin(self._delta))

        return (df6_dvx, df6_dvy, df6_domega, df6_ddelta)


    def _discretized_linearized_model(self, Xbar_k, Ubar_k, sx, su, Ts) -> tuple:

        # Jacobians of the system model
        df1_dphi, df1_dvx, df1_dvy = self._getFirstJacobians()
        df2_dphi, df2_dvx, df2_dvy = self._getSecondJacobians()
        df3_domega = 1
        df4_dvx, df4_dvy, df4_domega, df4_dD, df4_ddelta = self._getForthJacobians()
        df5_dvx, df5_dvy, df5_domega, df5_ddelta = self._getFifthJacobians()
        df6_dvx, df6_dvy, df6_domega, df6_ddelta = self._getSixthJacobians()

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

        gc = self._f - Ac @ Xbar_k[:sx] - Bc @ Ubar_k[:su]

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
        # -1 => indexing last value of matrix / list / etc.
        Ad[-1, -1] = 1
        Bd[-1, -1] = Ts

        return (Ad, Bd, gd)