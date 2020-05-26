import numpy as np
from .controller import Controller
from models.manipulator_model import ManiuplatorModel


class MMAController(Controller):
    def __init__(self, Tp):
        # TODO: Fill the list self.models with 3 models of 2DOF manipulators with different m3 and r3
        # Use parameters from manipulators/mm_planar_2dof.py
        self.models = [ManiuplatorModel(Tp, m3=0.1, r3=0.1), ManiuplatorModel(Tp, m3=0.01, r3=0.01), ManiuplatorModel(Tp, m3=1, r3=0.3)]
        self.i = 0
        self.kd = np.diag((1,1)) * [1, -3]
        self.kp = np.diag((1,1)) * [15, 5]

    def choose_model(self, x, u, x_dot):
        # TODO: Implement procedure of choosing the best fitting model from self.models (by setting self.i)
        error_list = []
        for model in self.models:
            xm_dot = model.M(x).dot(u) + model.C(x).dot(x[2:].reshape(-1, 1))
            error_list.append(x_dot[:2].reshape(-1, 1) - xm_dot)

        error_list = np.sum(error_list, axis=1)

        self.i = np.argmin(error_list)
        #self.i = 0

    def calculate_control(self, x, q_d, q_d_dot, q_dd_dot):
        #v = desired_q_ddot  # TODO: Add Feedback
        v = q_dd_dot + self.kd.dot(x[2:].reshape(-1, 1)-q_d_dot.reshape(-1, 1)) + self.kp.dot(x[:2].reshape(-1, 1)-q_d.reshape(-1, 1))
        q_dot = x[2:, np.newaxis]
        M = self.models[self.i].M(x)
        return M @ (v + np.linalg.inv(M) @ self.models[self.i].C(x) @ q_dot)