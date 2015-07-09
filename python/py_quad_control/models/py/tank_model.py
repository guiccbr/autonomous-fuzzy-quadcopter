# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4

# ------------------------ Imports ----------------------------------#
from math import sqrt

# ------------------------ Classes  ---------------------------------#
class WaterTank:

    def update(self, u):
        self.uk = u
        self.y = max(0.0, (self.y + self.t*(-sqrt(19.6*self.y) + self.uk)/((self.y**2)+1)))

    def __init__(self, y1, t):
        self.uk = 0 
        self.t = t
        self.y = y1

    def get_y(self):
        return self.y
