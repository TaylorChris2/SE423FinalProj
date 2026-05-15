import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
import time
import threading
import queue
import posix_ipc

from ctypes import *
from typing import Optional, Tuple
from collections import deque
from multiprocessing import shared_memory
from dynamics import EOM_kin_ca

import matplotlib.animation as animation
from matplotlib.patches import Circle

# Semaphore and shared memory paths
LIDAR_SEM_MUTEX_NAME = "sem-new-ladar-dist"
LIDAR_SHARED_MEM_NAME = "posix-shared-mem-ladar-dist"
ODOM_SEM_MUTEX_NAME = "sem-LVCOMApp-sendto"
ODOM_SHARED_MEM_NAME = "sharedmem-LVCOMApp-sendto"
CONTROL_SEM_MUTEX_NAME = "sem-LVCOMApp-readfrom"
CONTROL_SHARED_MEM_NAME = "sharedmem-LVCOMApp-readfrom"

# Data types for receiving odometry/waypoint and lidar data and for sending control data
N_BEAMS = 228
ODOM_DTYPE = np.dtype([
    ('x', '<f4'),
    ('y', '<f4'),
    ('theta', '<f4'),
    ('w_x', '<f4'),
    ('w_y', '<f4'),
    ('mpc_switch', '<f4'),
    ('unused', '<f4', (2,)),   # 32 bytes
])
LIDAR_DTYPE = np.dtype([
    ('points',         '<f4', (N_BEAMS,)),  # 1824 bytes
])
CONTROL_DTYPE = np.dtype([
    ('Vref1', '<f4'),
    ('Turnref1', '<f4'),
    ('Vref2', '<f4'),
    ('Turnref2', '<f4'),
    ('Vref3', '<f4'),
    ('Turnref3', '<f4'),
    ('flag', '<u4'),
    ('dt', '<f4'), # 32 bytes
])

class ShmRegion:
    """A POSIX shared-memory region viewed as a numpy structured scalar. As used in Marius SLAM code"""
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
        print("shm name:", self.name)
        print("dtype size:", dtype.itemsize)
        print("shm size:", self.shm.size)
        self.array = np.ndarray((1,), dtype=dtype, buffer=self.shm.buf)
    def close(self):
        try: self.shm.close()
        except Exception: pass
    def unlink(self):
        try: self.shm.unlink()
        except Exception: pass

class NamedSemaphore:
    """A POSIX named semaphore. Same name across producer and consumer. Adapted from Marius SLAM code"""
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
        """Drain the current semaphore count ro get the latest data from shared memory"""
        
        # Wait until at least one update exists
        try:
            self.sem.acquire(timeout)
        except posix_ipc.BusyError:
            return False

        # Iterate through the semaphore to drain
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
    """Drains the lidar semaphore, pushes copies of scans onto a queue. Adapted from Marius SLAM code"""
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
            if not self.sem.acquire_latest(timeout=0.2):
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

class OdomReader(threading.Thread):
    """Drains the odometry semaphore, pushes copies of scans onto a queue. Adapted from Marius SLAM code LidarReader"""
    def __init__(self, shm: ShmRegion, sem: NamedSemaphore, q: queue.LifoQueue):
        super().__init__(daemon=True, name="OdomReader")
        self.shm = shm
        self.sem = sem
        self.q = q
        self.last_seq = -1
        self.received = 0
        self.dropped = 0
        self._stop = threading.Event()
        self.ft_per_pixel = 8 / 39
    def stop(self): self._stop.set()
    def run(self):
        view = self.shm.array
        while not self._stop.is_set():
            if not self.sem.acquire_latest(timeout=0.2):
                continue
            # Get odometry
            x = float(view['x'][0])
            y = float(view['y'][0])
            theta = float(view['theta'][0])
            
            # Get waypoints transformed from map coordinates in pixel space to robot coordinates in ft
            w_x = (float(view['w_x'][0]) * self.ft_per_pixel - 286.18)
            w_y = (float(view['w_y'][0]) * self.ft_per_pixel  - 332.305)
 
            # Control when MPC runs
            mpc_switch = int(view['mpc_switch'][0])
            
            self.received += 1
            try:
                self.q.put_nowait((x, y, theta, w_x, w_y, mpc_switch))
            except queue.Full:
                try: self.q.get_nowait()
                except queue.Empty: pass
                try: self.q.put_nowait((x, y, theta, w_x, w_y, mpc_switch))
                except queue.Full: pass
                self.dropped += 1
        
class MPC:

    def __init__(self, lidar_shm_name: str, lidar_sem_name: str, odom_shm_name: str, odom_sem_name: str, control_shm_name: str, control_sem_name: str):
        """Casadi-based MPC class to send Vref and Turnref to the red board"""
        
        # Initialize optimization problem
        self.opti = ca.Opti()

        self.NX = 3 # State dimensions
        self.NU = 2 # Control dimensions
        self.N = 20 # Horizon length
        self.dt = 0.25 # Horizon timestep

        # Control bounds
        self.v_max = 2 # ft/s
        self.omega_max = 2 # rad/s
        self.du_max = 10 # ft/s^2, rad/s^2

        # Cost matrices
        self.Q = ca.diag(ca.DM([50.0, 50.0, 10.0]))
        self.R = ca.diag(ca.DM([10, 50]))
        
        self.safe_radius = 1 #ft, obstacle inflation radius
        self.obst_alpha = 1e1 #Obstacle cost gain
        self.max_obsts = 5 #Max obstacles to be processed in the MPC
        self.lidar_deg_res = 1.05 #deg
        self.lidar_zero_idx = 113
        self.lidar_offset = 0.75 #ft, Distance from robot to LiDAR center
        self.lidar_tol = 5 #LiDAR points to throw away
        
        # Initial pose and control
        self.robot_x = 0
        self.robot_y = 0
        self.robot_theta = 0
        self.mpc_switch = 0
        self.waypoint_x = 0
        self.waypoint_y = 0
        self.Xprev = np.zeros((self.NX, self.N + 1))
        self.Uprev = np.zeros((self.NU, self.N))
        self.X_sol = np.zeros((self.NX, self.N + 1))
        self.U_sol = np.zeros((self.NU, self.N))
        
        # Visualization state
        self.latest_obstacles = np.zeros((2, self.max_obsts))
        self.latest_path = np.zeros((self.NX, self.N + 1))

        # Shared memory and semaphore args
        self.lidar_shm_name = lidar_shm_name
        self.lidar_sem_name = lidar_sem_name
        self.odom_shm_name = odom_shm_name
        self.odom_sem_name = odom_sem_name
        self.control_shm_name = control_shm_name
        self.control_sem_name = control_sem_name

        # Begin communication threads and setup the MPC QP and constraints
        self.init_threads()
        self.mpc_setup_problem()

    def init_threads(self):
        """Starts communication threads for receiving LiDAR and odometry/waypoint data and for sending control data"""
        
        self.seq_counter = 0

        self.lidar_shm = ShmRegion(self.lidar_shm_name, LIDAR_DTYPE, create=False)
        self.lidar_sem = NamedSemaphore(self.lidar_sem_name)
        self.lidar_q   = queue.LifoQueue(maxsize=4) #LifoQueue grabs the data most recently put into the queue

        self.odom_shm  = ShmRegion(self.odom_shm_name,  ODOM_DTYPE,  create=False)
        self.odom_sem  = NamedSemaphore(self.odom_sem_name)
        self.odom_q    = queue.LifoQueue(maxsize=16)

        self.lidar_reader = LidarReader(self.lidar_shm, self.lidar_sem, self.lidar_q)
        self.odom_reader  = OdomReader(self.odom_shm, self.odom_sem, self.odom_q)
        self._stop = threading.Event()
        
        self.control_shm  = ShmRegion(self.control_shm_name,  CONTROL_DTYPE,  create=False)
        self.control_sem  = NamedSemaphore(self.control_sem_name)
    
    def start_threads(self):
        """Start MPC communication threads"""
        self.lidar_reader.start()
        self.odom_reader.start()
    
    def stop_threads(self):
        """Stop and close MPC communication threads. Adapted from Marius SLAM code"""
        self._stop.set()
        self.lidar_reader.stop()
        self.odom_reader.stop()

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
        """Create the MPC QP with added obstacle cost and dynamics constraints"""

        # Decision variables
        self.X = self.opti.variable(self.NX, self.N + 1)  # States
        self.U = self.opti.variable(self.NU, self.N)  # Controls

        self.x0 = self.opti.parameter(self.NX)  # Initial state
        self.xd = self.opti.parameter(self.NX)  # Waypoint

        # Dynamics constraints (xdot = Ax + Bu)
        for k in range(self.N):
            A, B = EOM_kin_ca(self.X[:, k], self.dt) # Get linearized robot dynamics across the horizon
            x_next = ca.mtimes([A, self.X[:, k]]) + ca.mtimes([B, self.U[:, k]])
            self.opti.subject_to(self.X[:, k + 1] == x_next)

        # Initial state constraint
        self.opti.subject_to(self.X[:, 0] == self.x0)

        # Control bounds constraint
        self.opti.subject_to(self.opti.bounded(-self.v_max, self.U[0, :], self.v_max))
        self.opti.subject_to(self.opti.bounded(-self.omega_max, self.U[1, :], self.omega_max))
        
        # Control rate bounds constraints (max dU/dt)
        for k in range(self.N-1):    
            delta = self.U[:, k+1] - self.U[:, k]         # 2x1    
            self.opti.subject_to(self.opti.bounded(-self.du_max, delta / self.dt, self.du_max))

        # Standard Qudratic Cost Function (xQx' + uRu')
        self.cost = 0
        for k in range(self.N):
            self.cost += ca.mtimes([(self.X[:, k] - self.xd).T, self.Q, (self.X[:, k] - self.xd)])
        for k in range(self.N):
            self.cost += ca.mtimes([self.U[:, k].T, self.R, self.U[:, k]])

        # Parameter to store obstacle positions
        self.obst_pos = self.opti.parameter(2, self.max_obsts)

        # Add obstacle costs along the horizon
        for k in range(self.N + 1):
            pos_k = self.X[:2, k]
            
            # Get robot distance from obstacles
            diff = self.obst_pos - ca.repmat(pos_k, 1, self.max_obsts)
            dist_sq = ca.sum1(diff * diff)
            dist = ca.sqrt(dist_sq)
            
            # Apply tuned inverse squared cost function to penalize crossing the safe radius but nowhere else
            margin = ca.fmax(dist - self.safe_radius, 1e-4)
            self.cost += ca.sum2(self.obst_alpha / margin**2)

        self.opti.minimize(self.cost)

        # Configure solver
        solver_opts = {
            "ipopt.max_iter": 50,
            "ipopt.tol": 1e-3,
            "ipopt.print_level": 0,
            "ipopt.linear_solver": "mumps",
            "ipopt.mu_strategy": "adaptive",
            "ipopt.warm_start_init_point": "yes",
            #"ipopt.max_wall_time": 1.5
            #"print_time": 0,
            #"ipopt.sb": "yes",
            # "ipopt.warm_start_mult_bound_push": 1e-6
        }
        self.opti.solver("ipopt", solver_opts)

    def update_obstacles(self):
        """Filters LiDAR scans to the N-closest and minimum spaced obstacles"""

        try:
            scan = self.lidar_q.get(timeout=0.1)[self.lidar_tol:-self.lidar_tol] #Get toleranced scan
            scan = scan / 1000 * 3.28084 # Convert LiDAR data from mm to ft
            
            # Get only scans greater than 0.05 ft away
            valid = np.isfinite(scan) & (scan > 0.05)
            valid_idx = np.where(valid)[0]
            valid_scan = scan[valid]

            # Sort points by distance
            sorted_order = np.argsort(valid_scan)
            candidate_idx = valid_idx[sorted_order]
            candidate_scan = valid_scan[sorted_order]

            selected_points = []
            selected_idx = []

            min_spacing = self.safe_radius # Minimum spacing between obstacles chosen to avoid

            for i in range(len(candidate_scan)):

                r = candidate_scan[i]

                # Get theta for each lidar point
                theta = np.deg2rad(
                    self.lidar_deg_res *
                    (candidate_idx[i] - self.lidar_zero_idx)
                )

                # Convert lidar points to xy
                px = r * np.cos(theta)
                py = r * np.sin(theta)
                candidate_pt = np.array([px, py])

                # Always keep first point
                if len(selected_points) == 0:
                    selected_points.append(candidate_pt)
                    selected_idx.append(i)
                    continue

                # Distance to previously selected points
                keep = True
                for prev_pt in selected_points:

                    d = np.linalg.norm(candidate_pt - prev_pt) # Distance between lidar points

                    if d < min_spacing: # Iterate through other minimum distance points until minimum spacing reached
                        keep = False
                        break

                if keep:
                    selected_points.append(candidate_pt)
                    selected_idx.append(i)

                if len(selected_points) >= self.max_obsts: # Obstacle array filled
                    break


            # Build obstacle position array (filled with 1e6 for empty indeces)
            min_scan_xy = np.full((2, self.max_obsts), 1e6)
            for i, pt in enumerate(selected_points):
                min_scan_xy[0, i] = pt[0]
                min_scan_xy[1, i] = pt[1]

            # Transform to world frame
            min_scan_xy = self.transform_lidar(min_scan_xy)

            # SWave obstacles for plotting
            self.latest_obstacles = min_scan_xy.copy()

            # Update obstacle problem parameter
            self.opti.set_value(self.obst_pos, min_scan_xy)
            
        except queue.Empty:
            pass

    def update_robot_pose_and_waypoints(self):
        """Get current robot pose and waypoints and mpc status"""
        
        try:
            data = self.odom_q.get(timeout=0.1)
            
            # Get odometry and waypoint data and store value of the switch
            self.robot_x = data[0]
            self.robot_y = data[1]
            self.robot_theta = data[2]
            self.waypoint_x = data[3]
            self.waypoint_y = data[4]
            self.mpc_switch = data[5]
            
            self.opti.set_value(self.x0, np.array([self.robot_x, self.robot_y, self.robot_theta]).reshape(3,1)) # Set next initial condition to current robot pose
            #print("Set x0 to", np.array([self.robot_x, self.robot_y, self.robot_theta]).reshape(3,1))
            self.opti.set_value(self.xd, np.array([self.waypoint_x, self.waypoint_y, 0.0]).reshape(3,1)) # Set current waypoint

        except queue.Empty:
            pass

    def solve_mpc(self):
        """Calls IPOPT solver to solve the MPC problem"""

        # Warm start MPC solutions with the previous solution
        self.opti.set_initial(self.X, self.Xprev)
        self.opti.set_initial(self.U, self.Uprev)

        try:

            sol = self.opti.solve_limited() # Solve function with timeout capability

            # Get solver status and raise error if failed
            stats = self.opti.stats()
            success = stats["success"]
            status = stats["return_status"]
            print(status)

            if not success:
                raise RuntimeError(status)

            # Extract solution
            X_sol = sol.value(self.X)
            U_sol = sol.value(self.U)

            # Save solutions
            self.Xprev = X_sol
            self.Uprev = U_sol
            self.X_sol = X_sol
            self.U_sol = U_sol

        except Exception as e:

            print("MPC failed:", e)

            # Reset warm start from bad solution
            self.opti.set_initial(self.X, 0)
            self.opti.set_initial(self.U, 0)
        
            # Stop robot if solve failed or took too long
            self.U_sol = np.zeros((self.NU, self.N))
            
            print("Timeout")
                

    def transform_lidar(self, min_scan):
        """Transforms LiDAR points from the robot frame to the world frame. Adapted from red board code"""

        theta_std = self.robot_theta
        R = np.array([[np.cos(theta_std), -np.sin(theta_std)],
                      [np.sin(theta_std), np.cos(theta_std)]])
        lidar_origin = np.array([self.robot_x, self.robot_y]).reshape(2,1) + R @ np.array([0, self.lidar_offset]).reshape(2,1)
        min_scan_world = R @ min_scan + lidar_origin
        return min_scan_world

    def publish_control(self, U: np.ndarray):
        """Sends solved MPC control values to SerialComOpti"""
        
        shm = self.control_shm
        sem = self.control_sem
        
        view = shm.array
        U = U.astype(np.float32)
        
        # Store the first three control values: one to apply immediately and the other two for caching if solving takes longer than dt
        view["Vref1"][0] = float(U[0,0])
        view["Turnref1"][0] = -float(U[1,0])
        view["Vref2"][0] = float(U[0,1])
        view["Turnref2"][0] = -float(U[1,1])
        view["Vref3"][0] = float(U[0,2])
        view["Turnref3"][0] = -float(U[1,2])
        print(view)

        # Flag for redboard to know if new control values received
        view['flag'][0] = 1
        
        # Provide MPC timstep as a delay between cached control values
        view['dt'][0] = int(1000 * self.dt)

        sem.release()

    def start_animation(self):
        """AI generated visualization code for debugging. Starts the plots  for robot position, osbtacle positions and inflation radii, and planned MPC path"""

        self.fig, self.ax = plt.subplots(figsize=(10, 10))

        # MPC path line
        self.path_line, = self.ax.plot(
            [], [],
            'b-',
            linewidth=2,
            label='MPC Path'
        )

        # Robot current position
        self.robot_point, = self.ax.plot(
            [], [],
            'ko',
            markersize=10,
            label='Robot'
        )

        # Goal point
        self.goal_point, = self.ax.plot(
            [], [],
            'gx',
            markersize=12,
            markeredgewidth=3,
            label='Goal'
        )

        # Robot heading line
        self.heading_line, = self.ax.plot(
            [],
            [],
            'k-',
            linewidth=2
        )

        # Obstacle scatter
        self.obstacle_scatter = self.ax.scatter(
            [],
            [],
            c='r',
            s=40,
            label='Obstacles'
        )

        # Inflation circles
        self.obstacle_circles = []

        self.ax.set_xlim(-10, 10)
        self.ax.set_ylim(-10, 10)

        self.ax.set_xlabel("X [ft]")
        self.ax.set_ylabel("Y [ft]")

        self.ax.set_title("Real-Time MPC + Lidar Visualization")

        self.ax.grid(True)
        #self.ax.legend()

        self.ax.set_aspect('equal')

        self.anim = animation.FuncAnimation(
            self.fig,
            self.update_animation,
            interval=50,
            blit=False,
            cache_frame_data=False
        )

        plt.show()
        
    def update_animation(self, frame):
        """AI generated visualization code for debugging. Updates and plots the current robot position, osbtacle positions and inflation radii, and planned MPC path"""

        # =========================
        # MPC PATH
        # =========================
        path = self.latest_path

        self.path_line.set_data(
            path[0, :],
            path[1, :]
        )

        # =========================
        # ROBOT POSITION
        # =========================
        self.robot_point.set_data(
            [self.robot_x],
            [self.robot_y]
        )

        # =========================
        # GOAL POSITION
        # =========================
        self.goal_point.set_data(
            [self.waypoint_x],
            [self.waypoint_y]
        )

        # =========================
        # ROBOT HEADING
        # =========================
        heading_len = 0.75

        hx = self.robot_x + heading_len * np.cos(self.robot_theta)
        hy = self.robot_y + heading_len * np.sin(self.robot_theta)

        self.heading_line.set_data(
            [self.robot_x, hx],
            [self.robot_y, hy]
        )

        # =========================
        # OBSTACLES
        # =========================
        obst = self.latest_obstacles

        self.obstacle_scatter.set_offsets(
            obst.T
        )

        # Remove old circles
        for circ in self.obstacle_circles:
            circ.remove()

        self.obstacle_circles = []

        # Draw inflation circles
        for i in range(obst.shape[1]):

            ox = obst[0, i]
            oy = obst[1, i]

            if ox == 0 and oy == 0:
                continue

            circ = Circle(
                (ox, oy),
                self.safe_radius,
                color='r',
                fill=False,
                alpha=0.4,
                linewidth=1.5
            )

            self.ax.add_patch(circ)
            self.obstacle_circles.append(circ)

        # =========================
        # AUTO-SCALE VIEW
        # =========================
        all_x = [self.robot_x, self.waypoint_x]
        all_y = [self.robot_y, self.waypoint_y]

        if obst.size > 0:
            all_x.extend(obst[0, :])
            all_y.extend(obst[1, :])

        margin = 3

        self.ax.set_xlim(min(all_x) - margin, max(all_x) + margin)
        self.ax.set_ylim(min(all_y) - margin, max(all_y) + margin)

        return (
            self.path_line,
            self.robot_point,
            self.goal_point,
            self.heading_line,
            self.obstacle_scatter
        )


mpc = MPC(LIDAR_SHARED_MEM_NAME, LIDAR_SEM_MUTEX_NAME, ODOM_SHARED_MEM_NAME, ODOM_SEM_MUTEX_NAME, CONTROL_SHARED_MEM_NAME, CONTROL_SEM_MUTEX_NAME)

mpc.start_threads()

def mpc_loop():

    while True:
    
        mpc.update_robot_pose_and_waypoints() #Always update odometry/waypoints to stay up-to-date with the mpc switch
        
        if mpc.mpc_switch == 1:

            # Get osbtacles and publish control if MPC is on
            mpc.update_obstacles()
            mpc.solve_mpc()
            mpc.publish_control(mpc.U_sol[:, 0:3])

            #time.sleep(0.02)
        else:  
            
            # Send 0 control to stop robot while MPC is off
            mpc.U_sol = np.zeros((mpc.NU, mpc.N))
            mpc.publish_control(mpc.U_sol[:, 0:3])
            
            print("off", mpc.mpc_switch)
            time.sleep(0.1)

loop_thread = threading.Thread(
    target=mpc_loop,
    daemon=True
)

loop_thread.start()

# Start visualization
mpc.start_animation()

