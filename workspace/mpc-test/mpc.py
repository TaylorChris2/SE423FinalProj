import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
import time

from dynamics import EOM_kin_ca, EOM_kin
from scipy.linalg import solve_discrete_are

N = 20

class MPC:

    def __init__(self, xd_val):
        self.opti = ca.Opti()

        self.NX = 3
        self.NU = 2
        self.N = N
        self.dt = 0.25
        self.r = 0.05

        # Control bounds
        self.v_l_max = 0.3
        self.v_r_max = 0.3
        self.du_max = 0.3

         # Decision variables
        self.X = self.opti.variable(self.NX, self.N + 1)  # States
        self.U = self.opti.variable(self.NU, self.N)  # Controls

        self.x0 = self.opti.parameter(self.NX)  # Initial state
        self.xd = self.opti.parameter(self.NX, self.N + 1)  # Reference

        # Dynamics constraints
        for k in range(self.N):
            A, B = EOM_kin_ca(self.X[:, k], self.dt, self.r)
            x_next = ca.mtimes([A, self.X[:, k]]) + ca.mtimes([B, self.U[:, k]])
            self.opti.subject_to(self.X[:, k + 1] == x_next)

        # Initial state constraint
        self.opti.subject_to(self.X[:, 0] == self.x0)

        # Control bounds
        self.opti.subject_to(self.opti.bounded(-self.v_l_max, self.U[0, :], self.v_l_max))
        self.opti.subject_to(self.opti.bounded(-self.v_r_max, self.U[1, :], self.v_r_max))
        for k in range(N-1):    
            delta = self.U[:, k+1] - self.U[:, k]         # 2x1    
            self.opti.subject_to(self.opti.bounded(-self.du_max, delta / self.dt, self.du_max))

        # Quadratic cost
        Q = ca.diag(ca.DM([1000.0, 1000.0, 10.0]))   # Penalize x,y error more
        R = ca.diag(ca.DM([5, 5]))
        
        cost = 0
        for k in range(self.N):
            cost += ca.mtimes([(self.X[:, k] - self.xd[:,k]).T, Q, (self.X[:, k] - self.xd[:,k])])
        for k in range(self.N):
            cost += ca.mtimes([self.U[:, k].T, R, self.U[:, k]])

        # Terminal cost
        A, B = EOM_kin(xd_val[-1,:], self.dt, self.r)
        err_T = self.X[:, self.N] - self.xd[:,-1]
        Q_T = np.diag([1000.0, 1000.0, 10.0])
        R_T = np.diag([1, 1])
        # P = solve_discrete_are(A, B, Q_T, R_T)
        # cost += ca.mtimes([err_T.T, P, err_T])

        # self.obst_num = 200
        # self.obst_pos = self.opti.parameter(2 * self.obst_num)  # 2D obstacle position (x,y only)
        # self.safe_radius = 0.1
        # self.weight = 50

        # # In your constraint loop, add per-timestep obstacle avoidance
        # for k in range(self.N + 1):
        #     for z in range(0,self.obst_num):
        #         dx = self.X[0, k] - self.obst_pos[2*z]
        #         dy = self.X[1, k] - self.obst_pos[2*z+1]
        #         dist_sq = dx**2 + dy**2
                
        #         margin = ca.sqrt(dist_sq) - self.safe_radius
        #         cost += (ca.exp(1 / (self.weight * ca.fmax(margin, 1e-4))) - 1)

        self.obst_num = 4
        self.safe_radius = 0.1
        self.weight = 100

        # Store obstacles as (2 x obst_num) parameter for vectorized ops
        self.obst_pos = self.opti.parameter(2, self.obst_num)

        obst_cost = 0

        for k in range(self.N + 1):
            # Robot position at timestep k: (2 x 1)
            pos_k = self.X[:2, k]  # shape (2,)

            # Broadcast: (2 x obst_num) - (2 x 1) = (2 x obst_num)
            diff = self.obst_pos - ca.repmat(pos_k, 1, self.obst_num)

            # Squared distances: (1 x obst_num)
            dist_sq = ca.sum1(diff * diff)  # sum over rows (x and y)

            # Vectorized margin
            dist = ca.sqrt(dist_sq)
            
            
            margin = ca.fmax(dist - self.safe_radius, 1e-4)
            # Sum cost over all obstacles at this timestep
            # obst_cost += ca.sum2(ca.exp(1.0 / (self.weight * margin)) - 1)
            alpha = 1e-3
            obst_cost += ca.sum2(alpha / margin**2)

        cost += obst_cost
        
        self.opti.minimize(cost)

         # Configure solver
        solver_opts = {
            "ipopt.max_iter": 1e3,
            "ipopt.tol": 1e-5,
            "ipopt.print_level": 0,
            "ipopt.linear_solver": "mumps",
            "ipopt.mu_strategy": "adaptive",
            "ipopt.warm_start_init_point": "yes",
            # "ipopt.warm_start_mult_bound_push": 1e-6
        }
        self.opti.solver("ipopt", solver_opts)

    def _f(self, x, u, r):
        """True nonlinear dynamics: dx/dt = f(x,u)"""
        theta = x[2]
        v_l, v_r = u[0], u[1]
        return ca.vertcat(
            (v_l + v_r) / 2 * ca.cos(theta),
            (v_l + v_r) / 2 * ca.sin(theta),
            (v_l - v_r) / (2 * r)
        )

    def _rk4(self, x, u):
        """4th-order Runge-Kutta integration."""
        dt = self.dt
        r = self.r
        k1 = self._f(x, u, r)
        k2 = self._f(x + dt/2 * k1, u, r)
        k3 = self._f(x + dt/2 * k2, u, r)
        k4 = self._f(x + dt * k3, u, r)
        return x + (dt / 6) * (k1 + 2*k2 + 2*k3 + k4)

    def _rk4_np(self, x, u):
        """Numpy RK4 for simulation."""
        dt = self.dt
        r = self.r
        theta = x[2]
        def f(x, u, r):
            return np.array([(u[0]+u[1])/2*np.cos(x[2]), (u[0]+u[1])/2*np.sin(x[2]), (u[1]-u[0])/(2*r)])
        k1 = f(x, u, r)
        k2 = f(x + dt/2*k1, u, r)
        k3 = f(x + dt/2*k2, u, r)
        k4 = f(x + dt*k3, u, r)
        return x + (dt/6)*(k1 + 2*k2 + 2*k3 + k4)

x_next = np.array([0, 0, 0])
x_d = np.array([1, 0.5, np.pi/4])
x_d = np.linspace(x_d, x_d, N + 1).T

mpc = MPC(x_d)

first_iter = True
iter_count = 1
X_opt = np.zeros((mpc.NX, mpc.N + 1))

x_total = [x_next]
u_total = [np.array([0,0])]

obst_pos = np.ones((mpc.obst_num,2))
obst_pos[:,0] = 0.4
obst_pos[:,1] = 0.1
obst_pos[0:4,0] = np.array([0.4, 0.6, 0.82, 1.15])
obst_pos[0:4,1] = np.array([0.1, 0.4, 0.29, 0.4])

start = time.time()
print(x_d[:,-1][0:2])

while np.linalg.norm(x_next[0:2] - x_d[:,-1][0:2]) > 3e-2 and iter_count <= 1e2:

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

    # obst_pos[1] -= 0.01 * iter_count
    mpc.opti.set_value(mpc.obst_pos, obst_pos.T)

    # Solve
    sol = mpc.opti.solve()

    # Extract solution
    X_opt = sol.value(mpc.X)  # Optimal states (NX x N+1)
    U_opt = sol.value(mpc.U)  # Optimal controls (NU x N)


    # A, B = EOM_kin(x_next, mpc.dt, mpc.r)
    # x_next = A @ x_next + B @ U_opt[:, 0]
    x_next = mpc._rk4_np(x_next, U_opt[:,0])
    print(x_next)
    
    iter_count += 1
    
    print(np.linalg.norm(x_next[0:2] - x_d[:,-1][0:2]))
    x_total.append(x_next)
    u_total.append(U_opt[:, 0])

print((time.time() - start),1000 * (time.time() - start)/iter_count)
# obst_pos[1] = 0.2 - 0.01 * iter_count

# ── Plotting ───────────────────────────────────────────────────────
x_total = np.array(x_total)
u_total = np.array(u_total)

fig, axes = plt.subplots(1, 3, figsize=(15, 4))

axes[0].plot(x_total[:, 0], x_total[:, 1], 'b-o', markersize=2, label="Path")
axes[0].plot(0, 0, 'go', markersize=10, label="Start")
for x in range(0,len(obst_pos)):
    axes[0].plot(obst_pos[x,0], obst_pos[x,1], 'ro', markersize=10)
    circle = plt.Circle((obst_pos[x,0], obst_pos[x,1]), mpc.safe_radius, color='g')
    axes[0].add_patch(circle)
axes[0].plot(x_d[0], x_d[1], 'r*', markersize=15, label="Goal")
axes[0].set_xlabel("x"); axes[0].set_ylabel("y")
axes[0].set_title("XY Trajectory"); axes[0].legend(); axes[0].grid(True)

axes[1].plot(x_total[:, 0], label="x")
axes[1].plot(x_total[:, 1], label="y")
axes[1].plot(x_total[:, 2], label="θ")
axes[1].set_xlabel("Step"); axes[1].set_title("States"); axes[1].legend(); axes[1].grid(True)

axes[2].plot(u_total[:, 0], label="v_l (m/s)")
axes[2].plot(u_total[:, 1], label="v_r (m/s)")
axes[2].set_xlabel("Step"); axes[2].set_title("Controls"); axes[2].legend(); axes[2].grid(True)

plt.tight_layout()
plt.show()
