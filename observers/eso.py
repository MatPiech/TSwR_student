import numpy as np


class ESO:
    def __init__(self, A, B, L):
        self.A = A
        self.B = B
        self.L = L

    def compute_dot(self, eso_estimates, q, u):
        e = q - eso_estimates[0]
        ### TODO: Please implement me
        z_dot = self.A.dot(eso_estimates.reshape(3,1)) + self.B.dot(u) + self.L.dot(e)
        return z_dot
