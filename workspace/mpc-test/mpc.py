import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
import time

from dynamics import EOM_kin_ca, EOM_kin
from scipy.linalg import solve_discrete_are

class MPC:

    def __init__(self, xd_val):
        self.opti = ca.Opti()

        self.NX = 3
        self.NU = 2
        self.N = 50
        self.dt = 0.05

        # Control bounds
        self.v_max = 1.0
        self.w_max = 2.0

         # Decision variables
        self.X = self.opti.variable(self.NX, self.N + 1)  # States
        self.U = self.opti.variable(self.NU, self.N)  # Controls

        self.x0 = self.opti.parameter(self.NX)  # Initial state
        self.xd = self.opti.parameter(self.NX)  # Reference

        # Dynamics constraints
        for k in range(self.N):
            A, B = EOM_kin_ca(self.X[:, k], self.dt)
            x_next = ca.mtimes([A, self.X[:, k]]) + ca.mtimes([B, self.U[:, k]])
            self.opti.subject_to(self.X[:, k + 1] == x_next)

        # Initial state constraint
        self.opti.subject_to(self.X[:, 0] == self.x0)

        # Control bounds
        self.opti.subject_to(self.opti.bounded(-self.v_max, self.U[0, :], self.v_max))
        self.opti.subject_to(self.opti.bounded(-self.w_max, self.U[1, :], self.w_max))

        # Quadratic cost
        Q = ca.diag(ca.DM([100.0, 100.0, 1.0]))   # Penalize x,y error more
        R = ca.diag(ca.DM([0.1, 0.05]))
        
        cost = 0
        for k in range(self.N + 1):
            cost += ca.mtimes([(self.X[:, k] - self.xd).T, Q, (self.X[:, k] - self.xd)])
        for k in range(self.N):
            cost += ca.mtimes([self.U[:, k].T, R, self.U[:, k]])

        # Terminal cost
        A, B = EOM_kin(xd_val, self.dt)
        err_T = self.X[:, self.N] - self.xd
        Q_T = np.diag([100.0, 100.0, 10.0])
        R_T = np.diag([0.1, 0.05])
        P = solve_discrete_are(A, B, Q_T, R_T)
        cost += ca.mtimes([err_T.T, P, err_T])
        
        self.opti.minimize(cost)

         # Configure solver
        solver_opts = {
            "ipopt.max_iter": 100,
            "ipopt.tol": 1e-4,
            "ipopt.print_level": 0
        }
        self.opti.solver("ipopt", solver_opts)

    def _f(self, x, u):
        """True nonlinear dynamics: dx/dt = f(x,u)"""
        theta = x[2]
        v, w = u[0], u[1]
        return ca.vertcat(
            v * ca.cos(theta),
            v * ca.sin(theta),
            w
        )

    def _rk4(self, x, u):
        """4th-order Runge-Kutta integration."""
        dt = self.dt
        k1 = self._f(x, u)
        k2 = self._f(x + dt/2 * k1, u)
        k3 = self._f(x + dt/2 * k2, u)
        k4 = self._f(x + dt * k3, u)
        return x + (dt / 6) * (k1 + 2*k2 + 2*k3 + k4)

    def _rk4_np(self, x, u):
        """Numpy RK4 for simulation."""
        dt = self.dt
        theta = x[2]
        def f(x, u):
            return np.array([u[0]*np.cos(x[2]), u[0]*np.sin(x[2]), u[1]])
        k1 = f(x, u)
        k2 = f(x + dt/2*k1, u)
        k3 = f(x + dt/2*k2, u)
        k4 = f(x + dt*k3, u)
        return x + (dt/6)*(k1 + 2*k2 + 2*k3 + k4)

x_next = np.array([0, 0, 0])
x_d = np.array([1, 0.5, np.pi/4])
mpc = MPC(x_d)

first_iter = True
iter_count = 1
X_opt = np.zeros((mpc.NX, mpc.N + 1))

x_total = []
u_total = []

start = time.time()

while np.linalg.norm(x_next[0:2] - x_d[0:2]) > 1e-2 and iter_count <= 1e3:

    print(iter_count)

    if first_iter:
        # Set initial state and reference
        mpc.opti.set_value(mpc.x0, np.array([0, 0, 0]))  # Initial state
        mpc.opti.set_value(mpc.xd, x_d)  # Reference state
        first_iter = False
    else:
        # Set initial state and reference
        mpc.opti.set_value(mpc.x0, x_next)  # Initial state
        mpc.opti.set_value(mpc.xd, x_d)  # Reference state
        mpc.opti.set_initial(mpc.X, X_opt)
        mpc.opti.set_initial(mpc.U, U_opt)

    # Solve
    sol = mpc.opti.solve()

    # Extract solution
    X_opt = sol.value(mpc.X)  # Optimal states (NX x N+1)
    U_opt = sol.value(mpc.U)  # Optimal controls (NU x N)


    A, B = EOM_kin(x_next, mpc.dt)
    # x_next = A @ x_next + B @ U_opt[:, 0]
    x_next = mpc._rk4_np(x_next, U_opt[:,0])
    
    iter_count += 1
    
    print(np.linalg.norm(x_next[0:2] - x_d[0:2]))
    x_total.append(x_next)
    u_total.append(U_opt[:, 0])

print(time.time() - start)

# ── Plotting ───────────────────────────────────────────────────────
x_total = np.array(x_total)
u_total = np.array(u_total)

fig, axes = plt.subplots(1, 3, figsize=(15, 4))

axes[0].plot(x_total[:, 0], x_total[:, 1], 'b-o', markersize=2, label="Path")
axes[0].plot(0, 0, 'go', markersize=10, label="Start")
axes[0].plot(x_d[0], x_d[1], 'r*', markersize=15, label="Goal")
axes[0].set_xlabel("x"); axes[0].set_ylabel("y")
axes[0].set_title("XY Trajectory"); axes[0].legend(); axes[0].grid(True)

axes[1].plot(x_total[:, 0], label="x")
axes[1].plot(x_total[:, 1], label="y")
axes[1].plot(x_total[:, 2], label="θ")
axes[1].set_xlabel("Step"); axes[1].set_title("States"); axes[1].legend(); axes[1].grid(True)

axes[2].plot(u_total[:, 0], label="v (m/s)")
axes[2].plot(u_total[:, 1], label="ω (rad/s)")
axes[2].set_xlabel("Step"); axes[2].set_title("Controls"); axes[2].legend(); axes[2].grid(True)

plt.tight_layout()
plt.show()
