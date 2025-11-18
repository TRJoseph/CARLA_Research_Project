import numpy as np
import cvxpy as cp


class MPCController:
    def __init__(self, model):
        self.model = model

    def calc_Jacobian(self, x, u):
        L_f = self.model.L_f
        L_r = self.model.L_r
        dt = 0.05

        psi = x[2]
        v   = x[3]
        delta = u[1]
        a   = u[0]

        # Jacobian of the system dynamics
        A = np.zeros((4, 4))
        B = np.zeros((4, 2))

        #############################################################################
        #                    TODO: Implement your code here                         #
        #############################################################################

        A = np.array([[1, 0, -dt*v*np.sin(psi+np.arctan((L_r*np.arctan(delta))/(L_f+L_r))), dt*np.cos(psi+np.arctan((L_r*np.arctan(delta))/(L_f+L_r)))],
                    [0, 1, dt*v*np.cos(psi+np.arctan((L_r*np.arctan(delta))/(L_f+L_r))), dt*np.sin(psi+np.arctan((L_r*np.arctan(delta))/(L_f+L_r)))],
                    [0, 0, 1, (dt*np.arctan(delta))/((((L_r**2*np.arctan(delta)**2))/((L_f+L_r)**2)+1)**(1/2)*(L_f+L_r))],
                    [0, 0, 0, 1]])
        
        
        B = np.array([
        [0, -(dt*L_r*v*np.sin(psi+np.arctan((L_r*np.arctan(delta))/(L_f+L_r))))/((delta**2+1)*((L_r**2*np.arctan(delta)**2)/((L_f+L_r)**2)+1)*(L_f+L_r))],
        [0, (dt*L_r*v*np.cos(psi+np.arctan((L_r*np.arctan(delta))/(L_f+L_r))))/((delta**2+1)*((L_r**2*np.arctan(delta)**2)/((L_f+L_r)**2)+1)*(L_f+L_r))],
        [0, (dt*v)/((delta**2+1)*((L_r**2*np.arctan(delta)**2)/((L_f+L_r)**2)+1)**(3/2)*(L_f+L_r))],
        [dt, 0]])

        #############################################################################
        #                            END OF YOUR CODE                               #
        #############################################################################

        return [A, B]


    def compute_control(self, state, ctrl, x0):
        len_state = state.shape[0]
        len_ctrl  = ctrl.shape[0]
        dim_state = state.shape[1]
        dim_ctrl  = ctrl.shape[1]

        n_u = len_ctrl * dim_ctrl
        n_x = len_state * dim_state
        n_var = n_u + n_x

        x = cp.Variable(n_var)

        states = x[:n_x].reshape((len_state, dim_state), order="C") 
        controls = x[n_x:].reshape((len_ctrl, dim_ctrl), order="C")   

        Q = np.eye(4) * 5
        R = np.eye(2) * 0.1  
        Pt = np.eye(4) * 3000

        P = cp.quad_form(states[len_state-1], Pt)
        q = 0

        # build q from 0 to N-1
        for i in range(len_state-1):

            Q_term = cp.quad_form(states[i, :], Q)

            R_term = cp.quad_form(controls[i, :], R)

            q += (Q_term + R_term)

        # cost is running + terminal
        cost = P + q


        # a_limit and delta_limit are vectors containing the upper and lower constraints
        # upper bounds 
        ub = np.array([self.model.a_lim[1], self.model.delta_lim[1]])
        # lower bounds
        lb = np.array([self.model.a_lim[0], self.model.delta_lim[0]])

        # define constraints
        constraints = []

        for i in range(len_state-1):
            a, b = self.calc_Jacobian(state[i, :], ctrl[i, :])

            constraints.append(states[i+1, :] == a @ states[i, :] + b @ controls[i, :])
            
            # append new control constraint for acceleration and steering angle
            constraints.append(controls[i, :] >= lb - ctrl[i, :])
            constraints.append(controls[i, :] <= ub - ctrl[i, :])
            
        # add the last initial difference constraint
        state_diff_init = x0 - state[0, :]
        constraints.append(states[0, :] == state_diff_init)

        # solve optimization problem
        prob = cp.Problem(cp.Minimize(cost), constraints=constraints)
        prob.solve(verbose=False, max_iter = 10000)

        u_act = x.value[n_x:n_x + dim_ctrl] + ctrl[0, :]
        return u_act