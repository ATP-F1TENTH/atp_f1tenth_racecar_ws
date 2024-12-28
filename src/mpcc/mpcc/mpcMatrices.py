
import numpy as np

class mpcMatrices(None):
    """
    Class for creating the mpc system matricies
    """

    def __init__(self) -> None:
        pass


    def get_mpc_matrices(self, traj, MPC_vars, ModelParams, borders, Xhor, Uhor, x0, u0):
        """
        Function to generate the necessary system and cost matrices for the MPC problem.
        Args:
            traj, MPC_vars, ModelParams, borders, Xhor, Uhor, x0, u0: Various input parameters.

        Returns:
            X, U, dU, info: Solution matrices and solver info.
        """
        
        # Cost scaling for numerics
        cost_scale = MPC_vars['costScale']
        
        # Initialize stage list
        stage = []
        
        # Generate MPCC problem
        # Initial state (including previous input for input rate cost)
        stage.append({'x0': x0, 'u0': u0})
        
        for i in range(MPC_vars['N']):
            Xk = Xhor[:, i]
            Uk = Uhor[:, i]
            
            # Generate quadratic state(-input) cost
            Qk = cost_scale * self.generate_h(traj, MPC_vars, ModelParams, Xk, i)
            
            # Quadratic rate input cost
            Rk = cost_scale * 2 * np.diag([MPC_vars['rdD'], MPC_vars['rdDelta'], MPC_vars['rdVtheta']])
            
            # Linear state(-input) cost
            fk = cost_scale * self.generate_f(traj, MPC_vars, ModelParams, Xk, i)
            
            # Linearized dynamics
            Ak, Bk, gk = self.get_equality_constraints(Xk, Uk, MPC_vars, ModelParams)
            
            # Linearized track constraints
            Ck, ug, lg = self.get_inequality_constraints(borders[max(i-1, 0)], MPC_vars, ModelParams)
            
            # Bounds
            lb, ub = self.get_bounds(MPC_vars, ModelParams)
            
            # Append to stage
            stage.append({'Qk': Qk, 'Rk': Rk, 'fk': fk, 'Ak': Ak, 'Bk': Bk, 'gk': gk,
                        'Ck': Ck, 'ug': ug, 'lg': lg, 'lb': lb, 'ub': ub})
        
        # Terminal stage (N+1)
        i = MPC_vars['N']
        Xk = Xhor[:, i]
        
        # Generate quadratic state(-input) cost
        Qk = cost_scale * self.generate_h(traj, MPC_vars, ModelParams, Xk, i)
        
        # Quadratic rate input cost
        Rk = cost_scale * 2 * np.diag([MPC_vars['rdD'], MPC_vars['rdDelta'], MPC_vars['rdVtheta']])
        
        # Linear state(-input) cost
        fk = cost_scale * self.generate_f(traj, MPC_vars, ModelParams, Xk, i)
        
        # Linearized track constraints
        #! different borders as in loop above
        Ck, ug, lg = self.get_inequality_constraints(borders[i-1], MPC_vars, ModelParams)
        
        # Bounds
        lb, ub = self.get_bounds(MPC_vars, ModelParams)
        
        # Append to stage
        #! Appending not all values as in for-loop
        stage.append({'Qk': Qk, 'Rk': Rk, 'fk': fk, 'Ck': Ck, 'ug': ug, 'lg': lg, 'lb': lb, 'ub': ub})
        
        """
        # Call solver interface
        interface = MPC_vars['interface']
        if interface == 'Yalmip':
            X, U, dU, info = yalmip_interface(stage, MPC_vars, ModelParams)
        elif interface == 'CVX':
            X, U, dU, info = cvx_interface(stage, MPC_vars, ModelParams)
        elif interface == 'hpipm':
            X, U, dU, info = hpipm_interface(stage, MPC_vars, ModelParams)
        elif interface == 'quadprog':
            X, U, dU, info = quadprog_interface(stage, MPC_vars, ModelParams)
        else:
            raise ValueError("Invalid optimization interface")
        
        
        return X, U, dU, info
        """


    def generate_h(self, pathinfo, MPC_vars, ModelParams, Xk, i):
        """
        Generates the cost matrix for the state and input.
        """
        
        Qtilde = self.generate_qtilde(pathinfo, MPC_vars, ModelParams, Xk, i)
        
        if i == MPC_vars['N'] + 1:
            Qtilde[ModelParams['stateindex_omega'], ModelParams['stateindex_omega']] = MPC_vars['qOmegaNmult'] * MPC_vars['qOmega']
        else:
            Qtilde[ModelParams['stateindex_omega'], ModelParams['stateindex_omega']] = MPC_vars['qOmega']
        
        # Make Qtilde symmetric
        Qtilde = 0.5 * (Qtilde + Qtilde.T)
        
        # Contouring lag-error and real input costs
        Qk = 2 * np.block([[Qtilde, np.zeros_like(Qtilde)], 
                        [np.zeros_like(Qtilde), np.diag([MPC_vars['rD'], MPC_vars['rDelta'], MPC_vars['rVtheta']])]])
        
        Qk = np.dot(np.dot(np.linalg.inv(MPC_vars['Tx']), Qk), np.linalg.inv(MPC_vars['Tu'])) + 1e-12 * np.eye(10)

        return Qk


    def generate_qtilde(self, pathinfo, MPC_vars, ModelParams, Xk, i):
        """
        Computes the quadratic term for the cost function.
        """
        
        if i == MPC_vars['N'] + 1:
            Q = np.diag([MPC_vars['qCNmult'] * MPC_vars['qC'], MPC_vars['qL']])
        else:
            Q = np.diag([MPC_vars['qC'], MPC_vars['qL']])
        
        theta_virt = np.mod(Xk[-1], pathinfo['ppx']['breaks'][-1])
        grad_eC, grad_eL = self.get_error_gradient(pathinfo, theta_virt, ModelParams, Xk[0], Xk[1])
        
        errorgrad = np.concatenate([grad_eC, grad_eL])
        Qtilde = errorgrad.T @ Q @ errorgrad
        return Qtilde


    def generate_f(self, pathinfo, MPC_vars, ModelParams, Xk, i):
        """
        Generates the linear cost term.
        """
        
        #! physical => where we currently are | virtual => calculated / forecast
        x_phys = Xk[0]
        y_phys = Xk[1]
        
        theta_virt = np.mod(Xk[-1], pathinfo['ppx']['breaks'][-1])
        eC, eL = self.get_errors(pathinfo, theta_virt, x_phys, y_phys)
        
        e = np.concatenate([eC, eL])
        grad_eC, grad_eL = self.get_error_gradient(pathinfo, theta_virt, ModelParams, x_phys, y_phys)
        
        grad_e = np.concatenate([grad_eC, grad_eL])
        
        if i == MPC_vars['N'] + 1:
            Q = np.diag([MPC_vars['qCNmult'] * MPC_vars['qC'], MPC_vars['qL']])
        else:
            Q = np.diag([MPC_vars['qC'], MPC_vars['qL']])
        
        fx = 2 * e.T @ Q @ grad_e - 2 * Xk.T @ grad_e.T @ Q @ grad_e
        fT = np.concatenate([fx, np.zeros(ModelParams['nu'] - 1), -MPC_vars['qVtheta']])
        
        f = np.dot(np.dot(np.linalg.inv(MPC_vars['Tx']), fT), np.linalg.inv(MPC_vars['Tu']))
        return f


    def get_equality_constraints(self, Xk, Uk, MPC_vars, ModelParams):
        """
        Computes the equality constraints.
        """
        nx = ModelParams['nx']
        nu = ModelParams['nu']
        
        # Linearize and discretize the nonlinear bicycle model
        Ad, Bd, gd = self.discretized_linearized_model(Xk, Uk, ModelParams, MPC_vars['Ts'])
        
        Ak = np.block([[np.dot(MPC_vars['Tx'], np.dot(Ad, MPC_vars['invTx'])), np.dot(MPC_vars['Tx'], np.dot(Bd, MPC_vars['invTu']))],
                    [np.zeros((nu, nx)), np.eye(nu)]])
        
        Bk = np.vstack([np.dot(MPC_vars['Tx'], np.dot(Bd, MPC_vars['invTu'])), np.eye(nu)])
        
        gk = np.vstack([np.dot(MPC_vars['Tx'], gd), np.zeros((nu, 1))])
        
        return Ak, Bk, gk


    def get_inequality_constraints(border, MPC_vars, ModelParams):
        """
        Computes the inequality constraints.
        """
        nx = ModelParams['nx']
        nu = ModelParams['nu']
        
        x1, y1, x2, y2 = border
        
        numer = -(x2 - x1)
        denom = (y2 - y1)
        
        dbmax = max(numer * x1 - denom * y1, numer * x2 - denom * y2)
        dbmin = min(numer * x1 - denom * y1, numer * x2 - denom * y2)
        
        Ck = np.zeros(nx + nu)
        Ck[0:2] = [numer, -denom]
        
        ug = dbmax
        lg = dbmin
        
        Ck = np.dot(Ck, np.block([[MPC_vars['invTx'], np.zeros((nx, nu))], [np.zeros((nu, nx)), MPC_vars['invTu']]]))
        
        return Ck, ug, lg


    def get_bounds(MPC_vars, ModelParams):
        """
        Returns the bounds for the optimization problem.
        """
        lb = MPC_vars['bounds'][:, 0]
        ub = MPC_vars['bounds'][:, 1]
        
        return lb, ub