import time
import casadi as cs
import numpy as np
import warnings
warnings.filterwarnings("ignore")


class MPCController():
    def __init__(self, dt=1e-1, horizon=10) -> None:
        # states vector x, x_dot, theta, theta_dot, psi, psi_dot
        self.x = cs.MX.sym('x', 6)
        self.r = cs.MX.sym('r', 1)  # reward
        self.p = cs.MX.sym('p', 2)  # actions
        self.dt = dt
        self.horizon = horizon
        # system parameters
        d = 0.5
        l = 0.4
        r = 0.2
        m_b = 10
        m_w = 1
        J = 1.40833
        g = 9.81
        K = 0.051
        I_2 = 0.15
        I_1 = 0.60833
        I_3 = 0.60833

        # defining coefficients
        D_IP1 = (m_b + 3 * m_w) * (m_b * d ** 2 + I_3) - \
            (m_b * d * cs.cos(self.x[2]) ** 2)
        D_IP2 = m_b * (d * cs.sin(self.x[2]) ** 2) + I_2 + \
            (4 * l**2 + r**2/2 + (l * cs.cos(self.x[4]) ** 2)) * m_w

        A24 = (m_b * d ** 2 + I_3) * m_b * d * cs.sin(self.x[2]) * self.x[3]
        A26 = (m_b * d**2 * (1 - cs.cos(self.x[2]) ** 2) +
               I_3) * m_b * d * cs.sin(self.x[2]) * self.x[5]/D_IP1
        A44 = - (m_b * d) ** 2 * \
            cs.sin(self.x[2]) * cs.cos(self.x[2]) * self.x[3] / D_IP1
        A46 = 3 * m_w * m_b * d ** 2 * \
            cs.sin(self.x[2]) * cs.cos(self.x[2]) * self.x[5] / D_IP1
        A66 = -m_b * d**2 * cs.sin(self.x[2]) * \
            cs.cos(self.x[2]) * self.x[3]/D_IP1
        B21 = B22 = (m_b * d**2 + I_3)/(D_IP1 * r)
        B41 = B42 = -m_b * d * cs.cos(self.x[2]) / (D_IP1 * r)
        B61 = l * cs.cos(self.x[4])/(D_IP1 * r)
        B62 = - B61
        #####

        # transition matrix
        A = cs.MX(6, 6)
        A[0, 1] = 1
        A[1, 3] = A24
        A[1, 5] = A26
        A[2, 3] = 1
        A[3, 3] = A44
        A[3, 5] = A46
        A[4, 5] = 1
        A[5, 5] = A66
        # control matrix
        B = cs.MX(6, 2)
        B[1, 0] = B21
        B[1, 1] = B22
        B[3, 0] = B41
        B[3, 1] = B42
        B[5, 0] = B61
        B[5, 1] = B62
        ref_state = cs.MX([0.2, 0, 0, 0, 0, 0])
        # print((ref_state.shape))
        self.Q = cs.diag(cs.MX([0, 1., 1., 1., 0., 1.]))
        self.R = cs.diag(cs.MX([0.1, 0.1]))
        self.reward_rhs = (
            self.x - ref_state).T @ self.Q @ (self.x - ref_state) + self.p.T @ self.R @ self.p
        self.system_rhs = A @ self.x + B @ self.p
        # print(self.system_rhs.shape)
        self.ode = {'x': cs.vertcat(self.x,
                                    self.r),
                    'p': self.p,
                    'ode': cs.vertcat(self.system_rhs,
                                      self.reward_rhs)
                    }
        self.F = cs.integrator('F', 'rk', self.ode, {'tf': self.dt})
        # print(A)
        self.opts = {
            'ipopt': {
                'max_iter': 100,
                'print_level': 0,
                'acceptable_tol': 1e-3,
                'acceptable_obj_change_tol': 1e-3,
                # 'max_cpu_time': 0.01
            },
            'print_time': 0
        }

    def solve(self, state):
        u = cs.MX.sym('u', self.horizon * 2)
        x = cs.vertcat(state, 0)
        for k in range(self.horizon):
            x = self.F(x0=x, p=u[2*k:2*k+2])["xf"]
        r = x[-1]
        nlp = {'x': u,
               'f': r,
               'g': u}
        solver = cs.nlpsol('solver', 'ipopt', nlp, self.opts)
        res = solver(x0=0., lbg=-10, ubg=10)
        return [float(res['x'][0]), float(res['x'][1])]


if __name__ == "__main__":
    ctr = MPCController()
    init_state = np.zeros(6)
    init_state[1] = -0.2

    for i in range(10):
        start_time = time.time()
        res = ctr.solve(init_state)
        print(time.time() - start_time)
        print(res)
