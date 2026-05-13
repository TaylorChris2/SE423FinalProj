from flask import Flask, render_template, request, jsonify
import navigator 
import numpy as np


import numpy as np
import posix_ipc
import time
from ctypes import *
from typing import Optional, Tuple



from multiprocessing import shared_memory


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



waypoint_seq = 0


app = Flask(__name__)


robot_position = [0, 0]
current_waypoint_index = 0
current_waypoints = []

WAYPOINT_SEM_NAME = "sem-LVCOMApp-sendto"
WAYPOINT_SHM_NAME = "sharedmem-LVCOMApp-sendto"
FEEDBACK_SEM_NAME = "sem-LVCOMApp-readfrom"
FEEDBACK_SHM_NAME = "sharedmem-LVCOMApp-readfrom"

WAYPOINT_START = 999.0
WAYPOINT_MIDDLE = 888.0
WAYPOINT_END = 777.0


STATUS_NAVIGATING = 1
STATUS_RELOCALIZE_WALL_FOLLOW = 10
STATUS_APRIL_TAG_VISION = 20
#Unused currently: - IMPLEMENT?
STATUS_PAUSED = 2
STATUS_ARRIVED = 3
STATUS_IDLE = 0

WAYPOINT_DTYPE = np.dtype([
    ('data', '<f4', (8,))
])

FEEDBACK_DTYPE = np.dtype([
    ('data', '<f4', (8,))
])

waypoint_shm = ShmRegion(
    WAYPOINT_SHM_NAME,
    WAYPOINT_DTYPE,
    create=True
)

waypoint_sem = NamedSemaphore(
    WAYPOINT_SEM_NAME,
    create=True
)

feedback_shm = ShmRegion(FEEDBACK_SHM_NAME, FEEDBACK_DTYPE, create=False)
feedback_sem = NamedSemaphore(FEEDBACK_SEM_NAME, create=False)

robot_position = [0, 0, 0]
current_waypoint_index = 0
navigation_state = STATUS_IDLE

def update_feedback():

    global robot_position
    global current_waypoint_index
    global navigation_state

    #Updates status and position from received waypoints
    print("UPDATE")
    try:
        print("try")
        
        if not feedback_sem.sem.acquire(timeout=0.1):
                print("timeout")
                return

        data = feedback_shm.array['data'][0]

        robot_position = [
            float(data[0]),
            float(data[1]),
            float(data[2])
        ]
        print(robot_position)

        current_waypoint_index = int(data[6])

        status = int(data[5])

        if status in {
            STATUS_NAVIGATING,
            STATUS_RELOCALIZE_WALL_FOLLOW,
            STATUS_APRIL_TAG_VISION,
            STATUS_ARRIVED,
            STATUS_PAUSED}:

            navigation_state = status


    except Exception as e:
        print("no work")
        print(e)
        pass


def sendWaypoints(waypoints):

    print("SEND START")

    global waypoint_seq

    total = len(waypoints)

    if total == 0:
        return

    waypoint_seq += 1

    for idx, wp in enumerate(waypoints):
        print("WAYPOINT: ",idx,wp)

        if idx == 0:
            flag = WAYPOINT_START

        elif idx == total - 1:
            flag = WAYPOINT_END

        else:
            flag = WAYPOINT_MIDDLE

        x = float(wp[0])
        y = float(wp[1])

        view = waypoint_shm.array

        view['data'][0,0] = flag
        view['data'][0,1] = waypoint_seq
        view['data'][0,2] = idx
        view['data'][0,3] = total
        view['data'][0,4] = x
        view['data'][0,5] = y
        view['data'][0,6] = 0
        view['data'][0,7] = 0

        waypoint_sem.release()

        time.sleep(0.005)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/navigate', methods=['POST'])
def navigate():
    global navigation_state
    global current_waypoints
    global current_waypoint_index

    data = request.json
    room = data.get('room')
        

    # Calculate A star given waypoints:
    waypoints = navigator.path_Astar(int(room))
    
    current_waypoints = waypoints
    current_waypoint_index = 0

    sendWaypoints(waypoints)

    # Red borad should receive waypoints and start navigation

    
    return jsonify({
        "success": True,
        "waypoints": waypoints,
        "state": navigation_state
    })


@app.route('/control', methods=['POST'])
def control():
    global navigation_state

    if navigation_state == STATUS_NAVIGATING:
        print("PAUSE")
        #Tell redboard to enter pause state
        #TODO: Implement another "special" send flag to trigger this on redboard


    elif navigation_state == STATUS_PAUSED:
        #Tell redboard to enter resume state
        print("RESUME")

    return jsonify({"state": navigation_state})

@app.route('/status')
def status():
    global navigation_state

    global robot_position
    global current_waypoint_index


    update_feedback()

    return jsonify({
        "state": navigation_state,
        "robot_position": robot_position,
        "current_waypoint_index": current_waypoint_index
    })

@app.route('/toggle', methods=['POST'])
def toggle():
    global navigation_active
    navigation_active = not navigation_active
    return jsonify({"active": navigation_active})


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=False)
