import numpy as np
from .controller import Controller
from models.manipulator_model import ManiuplatorModel


class MMAController(Controller):
    def __init__(self, Tp):
        # TODO: Fill the list self.models with 3 models of 2DOF manipulators with different m3 and r3
        # Use parameters from manipulators/mm_planar_2dof.py
        self.i = 0
        
        self.models = [
            ManiuplatorModel(Tp, m3=0.1, r3=0.05), 
            ManiuplatorModel(Tp, m3=0.01, r3=0.01), 
            ManiuplatorModel(Tp, m3=1., r3=0.3)
            ]
        
        self.Kd = np.diag((1, 1)) * [1.0, -1.0]
        self.Kp = np.diag((1, 1)) * [1.3, 1.2]

    def choose_model(self, x, u, x_dot):
        # TODO: Implement procedure of choosing the best fitting model from self.models (by setting self.i)
        error = np.zeros(len(self.models))

        for i, model in enumerate(self.models):
            invM = np.linalg.inv(model.M(x))

            M = np.concatenate([np.zeros((2, 2)), invM], 0)
            C = np.concatenate([np.eye(2), -invM.dot(model.C(x))], 0)

            x_m = M.dot(u) + C.dot(x[2:].reshape(-1, 1))

            error[i] = np.sum(np.abs(x_m - x_dot))

        new_model = np.argmin(error)

        self.i = new_model

    def calculate_control(self, x, q_d, q_d_dot, q_dd_dot):
        #v = desired_q_ddot  # TODO: Add Feedback
        v = q_dd_dot + self.Kd.dot(x[2:].reshape(-1, 1)-q_d_dot.reshape(-1, 1)) + self.Kp.dot(x[:2].reshape(-1, 1)-q_d.reshape(-1, 1))
        q_dot = x[2:, np.newaxis]
        M = self.models[self.i].M(x)
        return M @ (v + np.linalg.inv(M) @ self.models[self.i].C(x) @ q_dot)