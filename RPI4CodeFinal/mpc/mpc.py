import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
import time
import threading
import queue

from ctypes import *
from typing import Optional
from sklearn.neighbors import KDTree
from multiprocessing import shared_memory
from dynamics import EOM_kin_ca

import posix_ipc

SEM_MUTEX_NAME = "sem-new-ladar-dist"
SHARED_MEM_NAME = "posix-shared-mem-ladar-dist"

N_BEAMS = 228
ODOM_DTYPE = np.dtype([
    ('x',         '<f8'),   # 8
    ('y',         '<f8'),   # 8
    ('theta',     '<f8'),   # 8  → 40 bytes
])
LIDAR_DTYPE = np.dtype([
    ('points',         '<f4', (N_BEAMS,)),  # 1824 → 1856 bytes total
])

class ShmRegion:
    """A POSIX shared-memory region viewed as a numpy structured scalar."""
    def __init__(self, name: str, dtype: np.dtype, create: bool = False):
        self.name = name
        size = dtype.itemsize
        if create:
            try:
                old = shared_memory.SharedMemory(name=name, create=False)
                old.close(); old.unlink()
            except FileNotFoundError:
                pass
            self.shm = shared_memory.SharedMemory(name=name, create=True, size=size)
        else:
            self.shm = shared_memory.SharedMemory(name=name, create=False)
        self.array = np.ndarray((1,), dtype=dtype, buffer=self.shm.buf)
    def close(self):
        try: self.shm.close()
        except Exception: pass
    def unlink(self):
        try: self.shm.unlink()
        except Exception: pass

class NamedSemaphore:
    """A POSIX named semaphore. Same name across producer and consumer."""
    def __init__(self, name: str, create: bool = False, initial: int = 0):
        self.name = name
        if create:
            try:
                posix_ipc.unlink_semaphore(name)
            except posix_ipc.ExistentialError:
                pass
            self.sem = posix_ipc.Semaphore(
                name, flags=posix_ipc.O_CREAT | posix_ipc.O_EXCL,
                initial_value=initial)
        else:
            self.sem = posix_ipc.Semaphore(name)
    
    def acquire(self, timeout: Optional[float] = None) -> bool:
        try:
            self.sem.acquire(timeout)
            return True
        except posix_ipc.BusyError:
            return False
    
    def acquire_latest(self, timeout: Optional[float] = None) -> bool:
        """
        Wait for at least one post,
        then drain all remaining semaphore counts by getting one.
        """
        # Wait until at least one update exists
        try:
            self.sem.acquire(timeout)
        except posix_ipc.BusyError:
            return False

        # Drain any backlog
        while True:
            try:
                self.sem.acquire(0)
            except posix_ipc.BusyError:
                break

        return True

    def release(self):
        self.sem.release()
    def release(self):
        self.sem.release()
    def close(self):
        try: self.sem.close()
        except Exception: pass
    def unlink(self):
        try: posix_ipc.unlink_semaphore(self.name)
        except posix_ipc.ExistentialError: pass

class LidarReader(threading.Thread):
    """Drains the lidar semaphore, pushes copies of scans onto a queue."""
    def __init__(self, shm: ShmRegion, sem: NamedSemaphore, q: queue.LifoQueue):
        super().__init__(daemon=True, name="LidarReader")
        self.shm = shm
        self.sem = sem
        self.q = q
        self.last_seq = -1
        self.received = 0
        self.dropped = 0
        self._stop = threading.Event()
    def stop(self): self._stop.set()
    def run(self):
        view = self.shm.array
        while not self._stop.is_set():
            if not self.sem.acquire(timeout=0.2):
                continue
            n     = N_BEAMS
            pts   = np.array(view['points'][0, :n], dtype=np.float32, copy=True)
            self.received += 1
            try:
                self.q.put_nowait(pts)
            except queue.Full:
                try: self.q.get_nowait()
                except queue.Empty: pass
                try: self.q.put_nowait(pts)
                except queue.Full: pass
                self.dropped += 1

    def get_min_objects(self):
        pass
        
        
class MPC:

    def __init__(self, lidar_shm_name: str, lidar_sem_name: str,):
        self.opti = ca.Opti()

        self.NX = 3
        self.NU = 2
        self.N = 20
        self.dt = 0.25
        self.r = 0.05

        # Control bounds
        self.v_l_max = 0.1
        self.v_r_max = 0.1
        self.du_max = 0.1

        # Cost matrices
        self.Q = ca.diag(ca.DM([1000.0, 1000.0, 10.0]))
        self.R = ca.diag(ca.DM([5, 5]))
        
        # Obstacle inflation radius and cost weight
        self.safe_radius = 0.1
        self.obst_alpha = 1e-3
        self.max_obsts = 10
        self.lidar_deg_res = 1.05 #deg
        self.lidar_zero_idx = 113
        self.lidar_offset = 0

        self.lidar_shm = ShmRegion(lidar_shm_name, LIDAR_DTYPE, create=False)
        self.lidar_sem = NamedSemaphore(lidar_sem_name)
        self.lidar_q   = queue.LifoQueue(maxsize=4)

        self.lidar_reader = LidarReader(self.lidar_shm, self.lidar_sem, self.lidar_q)
        self._stop = threading.Event()

        self.mpc_setup_problem()

    def start_threads(self):
        self.lidar_reader.start()
    
    def stop_threads(self):
        self._stop.set()
        self.lidar_reader.stop()

        # nudge readers blocked on semaphores
        try: self.lidar_sem.release()
        except Exception: pass
        try: self.odom_sem.release()
        except Exception: pass
        for t in (self._main_thread, self.lidar_reader, self.odom_reader, self.loop_worker):
            try: t.join(timeout=1.0)
            except Exception: pass
        self.lidar_shm.close()
        self.lidar_sem.close()

    def mpc_setup_problem(self):

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
        for k in range(self.N-1):    
            delta = self.U[:, k+1] - self.U[:, k]         # 2x1    
            self.opti.subject_to(self.opti.bounded(-self.du_max, delta / self.dt, self.du_max))

        # Qudratic Cost Function
        self.cost = 0
        for k in range(self.N):
            self.cost += ca.mtimes([(self.X[:, k] - self.xd[:,k]).T, self.Q, (self.X[:, k] - self.xd[:,k])])
        for k in range(self.N):
            self.cost += ca.mtimes([self.U[:, k].T, self.R, self.U[:, k]])

        # Store obstacles as (2 x obst_num) parameter for vectorized ops
        self.obst_pos = self.opti.parameter(2, self.max_obsts)

        for k in range(self.N + 1):
            # Robot position at timestep k: (2 x 1)
            pos_k = self.X[:2, k]  # shape (2,)
            # Broadcast: (2 x obst_num) - (2 x 1) = (2 x obst_num)
            diff = self.obst_pos - ca.repmat(pos_k, 1, self.max_obsts)
            # Squared distances: (1 x obst_num)
            dist_sq = ca.sum1(diff * diff)  # sum over rows (x and y)
            # Vectorized margin
            dist = ca.sqrt(dist_sq)
            
            margin = ca.fmax(dist - self.safe_radius, 1e-4)
            self.cost += ca.sum2(self.obst_alpha / margin**2)

        self.opti.minimize(self.cost)

        # Configure solver
        solver_opts = {
            "ipopt.max_iter": 1e3,
            "ipopt.tol": 1e-4,
            "ipopt.print_level": 0,
            "ipopt.linear_solver": "mumps",
            "ipopt.mu_strategy": "adaptive",
            "ipopt.warm_start_init_point": "yes",
            # "ipopt.warm_start_mult_bound_push": 1e-6
        }
        self.opti.solver("ipopt", solver_opts)

    def update_obstacles(self):
        
        try:
            scan = self.lidar_q.get(timeout=0.1)

            close_scan_idx = np.where(scan < 200000)
            close_scan = scan[close_scan_idx]

            if len(close_scan) == 0:
                return

            k = min(self.max_obsts, len(close_scan))

            idx = np.argpartition(close_scan, k - 1)[:k]
            print(idx)
            min_scan = close_scan[idx]

            min_scan_xy = np.zeros((2, self.max_obsts))

            for i in range(k):
                theta = np.deg2rad(self.lidar_deg_res * (idx[i] - self.lidar_zero_idx))
                min_scan_xy[0, i] = min_scan[i] * np.cos(theta)
                min_scan_xy[1, i] = min_scan[i] * np.sin(theta)

            min_scan_xy = self.transform_lidar(min_scan)
            self.opti.set_value(self.obst_pos, min_scan_xy)
            
        except queue.Empty:
            pass

    def update_robot_pose(self):
        pass

    def update_waypoints(self):
        pass

    def solve_mpc(self):
        pass

    def transform_lidar(self, min_scan):

        self.robot_theta = 0
        self.robot_x = 0
        self.robot_y = 0

        theta_std = self.robot_theta - np.pi / 2
        R = np.array([[np.cos(theta_std), -np.sin(theta_std)],
                      [np.sin(theta_std), np.cos(theta_std)]])
        lidar_origin = np.array([self.robot_x, self.robot_y]).reshape(2,1) + R @ np.array([0, self.lidar_offset]).reshape(2,1)
        min_scan_world = R @ min_scan + lidar_origin
        return min_scan_world


x_next = np.array([0, 0, 0])
x_d = np.array([1, 0.5, np.pi/4])
x_d = np.linspace(x_d, x_d, 21).T

mpc = MPC(SHARED_MEM_NAME, SEM_MUTEX_NAME)

first_iter = True
iter_count = 1
X_opt = np.zeros((mpc.NX, mpc.N + 1))

x_total = [x_next]
u_total = [np.array([0,0])]

start = time.time()

mpc.start_threads()

while np.linalg.norm(x_next[0:2] - x_d[:,-1][0:2]) > 3e-2 and iter_count <= 1e2:


    mpc.update_obstacles()
    time.sleep(0.01)
    '''print(iter_count)

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
    u_total.append(U_opt[:, 0])'''
