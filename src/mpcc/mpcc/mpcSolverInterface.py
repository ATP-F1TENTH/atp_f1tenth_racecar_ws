
from qpalm import Settings, Solver, Solution

class mpcSolver(None):

    def __init__(self, stage, MPC_vars, ModelParams) -> None:
        """Constructor of class mpcSolver"""
        self._solver = MPC_vars['interface']
        self.X = None
        self.U = None
        self.dU = None
        self.info = None


    def getResults(self) -> tuple:
        """Getter for necessary solver data"""
        return (self.X, self.U, self.dU, self.info)