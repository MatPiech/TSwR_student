import numpy as np
from trajectory_generators.trajectory_generator import TrajectoryGenerator


class Poly3(TrajectoryGenerator):
    def __init__(self, start_q, desired_q, T):
        self.T = T
        self.q_0 = start_q
        self.q_k = desired_q
        """
        Please implement the formulas for a_0 till a_3 using self.q_0 and self.q_k
        Assume that the velocities at start and end are zero.
        """
        self.a_0 = self.q_0[0]
        self.a_1 = 3*self.q_0[0] + self.q_0[1]
        self.a_2 = 3*self.q_k[0] - self.q_k[1]
        self.a_3 = self.q_k[0]

    def generate(self, t):
        """
        Implement trajectory generator for your manipulator.
        Positional trajectory should be a 3rd degree polynomial going from an initial state q_0 to desired state q_k.
        Remember to derive the first and second derivative of it also.
        Use following formula for the polynomial from the instruction.
        """
        t /= self.T
        q = self.a_3 * t**3 + self.a_2 * t**2 * (1 - t) + self.a_1 * t * (1 - t)**2 + self.a_0 * (1 - t)**3
        q_dot = t**2 * (3*self.a_3 - self.a_2) + t * (1 - t) * (2*self.a_2 - 2*self.a_1) + (1 - t)**2 * (self.a_1 - 3*self.a_0)
        q_ddot = 2*t * (3*self.a_3 - self.a_2) + (1 - 2*t) * (2*self.a_2 - 2*self.a_1) - 2*(1-t) * (self.a_1 - 3*self.a_0)
        
        return np.array([0, q]), np.array([0, q_dot]), np.array([0, q_ddot])
