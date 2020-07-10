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
        state = list(sym.symbols("x y z phi theta psi u v w p q r"))
        self.var["state"] = sym.Matrix(state)
        output = list(sym.symbols("du0 du1 du2 du3 du4 du5 du6 du7"))
        self.var["output"] = sym.Matrix(output)
        self.A, self.B = lmath.getSystem(self.var)
        self.subA = sym.lambdify(state, self.A, "numpy")
        self.subB = sym.lambdify(state, self.B, "numpy")
        #self.Q = np.zeros((12, 12))
        #for i in range(6):
        #    self.Q[i, i] = 1
        self.Q = np.eye(12)
        self.R = np.eye(8)

    # Act as a model predictive controller by substituting current state
    def get_state_space(self, state):
        return self.subA(*state), self.subB(*state), self.Q, self.R
