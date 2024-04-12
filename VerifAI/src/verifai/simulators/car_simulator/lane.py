# This file defines a lane
import numpy as np

class straight_lane():
    def __init__(self, p, q, w):
        self.p = np.array(p)
        self.q = np.array(q)
        self.w = w
        self.m = (self.q-self.p)/np.linalg.norm(self.p-self.q)
        self.n = np.array([-self.m[1], self.m[0]])

    def shifted(self, m):
        return straight_lane(self.p+self.n*self.w*m,
                             self.q+self.n*self.w*m, self.w)

    def dist(self, x):
        r = (x[0] - self.p[0]) * self.n[0] + (x[1] - self.p[1]) * self.n[1]
        return np.sqrt(r * r)


