import numpy as np
from models.manipulator_model import ManiuplatorModel
from .controller import Controller


class DummyController(Controller):
    def __init__(self, Tp):
        pass

    def calculate_control(self, x, q_d, q_d_dot, q_dd_dot):
        return q_dd_dot

    def choose_model(self, x, u, x_dot):
        pass
