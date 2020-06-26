import lqr_math as lmath
import yaml
import sympy as sym
import numpy as np
from thruster_manager import ThrusterManager


class SymbolicState:
    def __init__(self, config_filename, thruster_filename):
        with open(config_filename) as f:
            self.var = yaml.load(f)
        tm = ThrusterManager(thruster_filename)
        self.var["thruster"] = sym.Matrix(tm.wrenchmat)
        self.var["state"] = list(sym.Symbol("x y z phi theta psi u v w p q r"))
        self.var["output"] = list(sym.Symbol("du0 du1 du2 du3 du4 du5 du6 du7"))
        self.A, self.B = lmath.getSystem(self.var)
        self.subA = sym.lambdify(self.var["state"], self.A, "numpy")
        self.subB = sym.lambdify(self.var["state"], self.B, "numpy")
        self.Q = np.zeros(12)
        for i in range(6):
            self.Q[i][i] = 1
        self.R = np.eye(8)

    # Act as a model predictive controller by substituting current state
    def get_state_space(self, state):
        return self.subA(state), self.subB(state), self.Q, self.R
