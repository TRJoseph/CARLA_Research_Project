import numpy as np

class BicycleModel:

    """ 
    This is my attempt at modeling a Tesla Model 3 with a simplified bicycle model approach.
    Instead of tracking two separate parallel links from front to rear wheels, we can simpify the approach with one central link.
    This works well for highway driving.

    """
    def __init__(self, params):
        self.Dim_state = 4
        self.Dim_ctrl  = 2

        self.wheelbase = params["wheelbase"]
        self.L_f = params["L_f"]
        self.L_r = params["L_r"]
        self.a_lim = params["a_lim"]
        self.delta_lim = params["delta_lim"]
        self.h = 0.05

        self.initial_state = params["initial_state"]

    # calculates state dynamics for the bicycle model
    def Fun_dynamics_dt(self, x, u):

        xdot = np.zeros(4)        
        beta = np.arctan(self.L_r / (self.L_r + self.L_f) * np.arctan(u[1]))

        xdot[0] = x[3] * np.cos(x[2] + beta)
        xdot[1] = x[3] * np.sin(x[2] + beta)
        xdot[2] = x[3] / self.L_r * np.sin(beta)
        xdot[3] = u[0]
    
        xkp1 = x + xdot * self.h

        return xkp1
    