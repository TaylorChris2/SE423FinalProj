import heapq
import cv2
import numpy as np
img = cv2.imread("floorplan_bw.png", cv2.IMREAD_GRAYSCALE)

#converts coloration of pixel to white or black
# Convert to binary:
# black floor/open space = 1
# white walls = 0
val, binary = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)

#boolean arrays
walkable = binary == 0   # black pixels
walls = binary == 255    # white pixels

ft_per_pixel = 8 / 39
meters_per_pixel = ft_per_pixel * 0.3048
pixels_per_ft = 1 / ft_per_pixel

clearance_ft = 1.5
clearance_px = int(clearance_ft / ft_per_pixel)

kernel = np.ones((clearance_px, clearance_px), np.uint8)

walls_uint8 = walls.astype(np.uint8)
inflated_walls = cv2.dilate(walls_uint8, kernel, iterations=1)

#boolean array with True False values
walkable_safe = inflated_walls == 0

nodes = {
    "3080": (962, 690),
    "3082": (1186, 700),

    "3001": (836, 616),
    "3003": (834, 572),
    "3005": (831, 481),

    "3075": (833, 367),
    "3077": (830, 335),
    "3071": (1172, 340),
    "3073": (1173, 370),
    "3070": (1192, 276),
    "3072": (1187, 686),

    "3007": (753, 1069),
    "3009": (753, 1069),

    "3013": (635, 1085),
    "3015": (485, 1084),
    "3017": (331, 1086),

    "3032": (809, 224),
    "3034": (759, 227),
    "3036": (681, 222),
    "3021": (616, 223),

    "3025": (476, 205),
    "3027": (476, 205),

    "3038": (126, 222),
    "3026": (222, 228),
    "3024": (257, 232),
    "3022": (431, 232),
    "3018": (534, 234),
    "3016": (563, 230),
    "3014": (819, 320),
    "3030": (875, 217),
    "3051": (1130, 223),
}

maze = [
    'x','0','0','x','x','x','x','x','x','x','x',
    '0','0','0','0','x','x','0','x','x','0','x',
    '0','x','0','0','0','x','x','x','0','0','x',
    '0','x','0','x','0','x','x','x','x','0','x',
    '0','x','0','0','0','x','0','x','0','0','0',
    '0','x','0','x','x','x','x','x','x','x','0',
    '0','x','x','0','0','x','0','0','0','x','x',
    '0','0','x','x','0','x','0','x','0','x','0',
    '0','0','x','0','0','x','0','x','x','x','0',
    'x','x','x','x','0','x','0','x','0','0','0',
    'x','x','x','x','0','x','x','x','x','x','x',
    '0','0','0','0','0','0','0','x','0','0','0',
    '0','x','x','0','0','0','0','0','x','0','x',
    '0','0','0','x','0','0','x','0','0','0','0',
    '0','x','0','x','x','0','x','0','0','x','0',
    '0','0','0','0','0','0','x','0','x','x','0',
]

#Manhattan distance, far current cell from goal
def heuristic(row_curr, col_curr, row_goal, col_goal):
    return abs(row_goal - row_curr) + abs(col_goal - col_curr)

#inside bounds and not a barrier
def can_travel(row, col):
    if row < 0 or col < 0 or row >= walkable_safe.shape[0] or col >= walkable_safe.shape[1]:
        return False
    return walkable_safe[row, col]


def get_neighbors(row_curr, col_curr):
    neighbors = []
    #cycle through going up left right down
    #add cycle through going upleft upright downleft downright
    for changeRow, changeCol in [(-1,0),(0,-1),(0,1),(1,0),(-1,-1),(-1,1),(1,-1),(1,1)]:
        row, col = row_curr + changeRow, col_curr + changeCol
        if can_travel(row, col):
            #adds to list of cells that can be visited ned
            neighbors.append((row, col))
    return neighbors

# push the neihbors, then pop the lowest cost, check to see if its goal
# see if its lowest cot, push again, and keep reiterating until reach goal
def astar(row_start, col_start, row_end, col_end):
    if row_start == row_end and col_start == col_end:
        print("!!!!!!!! Already at goal, no A* needed !!!!!!!!")
        return None

    #keep track of every cell visited. parent, cost to reach, whether in open or closed set
    # node_track: (row,col) -> {'parent': (prow,pcol) or None, 'gone': int, 'state': 'open'|'closed'}
    node_track = {}

    # open set: min-heap of (f, gone, row, col)
    # gone is stored separately so ties on f can be broken and we always have the best dist from start
    open_set = []

    #cost from start to this cell
    gone_start = 0
    #estimate of cell to goal
    heur_start = heuristic(row_start, col_start, row_end, col_end)

    #sorts by lowest gone+heur first. then tie break with gone_start 
    heapq.heappush(open_set, (gone_start + heur_start, gone_start, row_start, col_start))
    node_track[(row_start, col_start)] = {'parent': None, 'gone': 0, 'state': 'open'}

    while open_set:
        #distTraveled from start
        #pops lowest cost cell from the heap
        totdist, distTravel, row, col = heapq.heappop(open_set)

        info = node_track.get((row, col))
        # skip stale heap entries, a better path was already found
        if info and info['state'] == 'closed':
            continue
        if info and info['gone'] < distTravel:
            continue
        #mark this node as closed: "done".
        node_track[(row, col)]['state'] = 'closed'

        for (neighborR, neighborC) in get_neighbors(row, col):
            if (neighborR, neighborC) == (row_end, col_end):
                node_track[(neighborR, neighborC)] = {'parent': (row, col), 'gone': distTravel + 1, 'state': 'closed'}
                return reconstruct_path(row_end, col_end, node_track)

            new_gone = distTravel + 1
            neighbor_info = node_track.get((neighborR, neighborC))
            #check if neighbor already on closed or open set
            if neighbor_info and neighbor_info['state'] == 'closed':
                continue
            if neighbor_info and neighbor_info['gone'] <= new_gone:
                continue
            #record this new cheaper path
            new_h = heuristic(neighborR, neighborC, row_end, col_end)
            node_track[(neighborR, neighborC)] = {'parent': (row, col), 'gone': new_gone, 'state': 'open'}
            heapq.heappush(open_set, (new_gone + new_h, new_gone, neighborR, neighborC))

    return None  # no path found


def reconstruct_path(row_end, col_end, node_track):
    path = []
    curr = (row_end, col_end)
    #goes from goal to start
    while curr is not None:
        path.append(curr)
        #makes the next one the parent for curr
        curr = node_track[curr]['parent']
    return path

def filter_waypoints(path):
    if len(path) <= 2:
        return path

    filtered = [path[0]] # always keep start

    for i in range(1, len(path) - 1):
        prev = path[i - 1]
        curr = path[i]
        nxt = path[i + 1]

        dr1 = curr[0] - prev[0]
        dc1 = curr[1] - prev[1]

        dr2 = nxt[0] - curr[0]
        dc2 = nxt[1] - curr[1]

        if dr1 != dr2 or dc1 != dc2:
            filtered.append(curr)

    filtered.append(path[-1]) # always keep end
    return filtered

def path_Astar(start_room, end_room):
    if start_room not in nodes:
        print(f"Room {start_room} not available")
        return None
    if end_room not in nodes:
        print(f"Room {end_room} not available")
        return None

    #x,y is col,row
    col_start, row_start = nodes[start_room]
    col_end, row_end = nodes[end_room]

    path = astar(row_start, col_start, row_end, col_end)
    if path is None:
        return None
    path = filter_waypoints(path)
    
    #path_temp = []
    #for x in path:
        #print(x)
        #path_temp.append((float(x[0]) * ft_per_pixel,float(x[1]) * ft_per_pixel))
    #for i in range(0,len(path_temp)):
        #path_temp[i] = (str(path_temp[i][0]), str(path_temp[i][1]))
    path_reversed = path[::-1]
    path_swapped = [(path_reversed[i][1],path_reversed[i][0]) for i in range(len(path_reversed))]
    return (path_swapped)



def main():
    while True:
        print("------------------------------")
        print("Menu:")
        print("Rooms:", list(nodes.keys()))
        print("8 - find path between two rooms")
        print("9 - exit")
        print("------------------------------")

        choice = int(input())

        if choice == 8:
            start = input("Enter start room number: ").strip()
            end   = input("Enter end room number: ").strip()
            path = path_Astar(start, end)
            if path:
                print(f"Path found: {len(path)} steps")
                print(f"Start pixel: {path[-1]}  ->  End pixel: {path[0]}")
            else:
                print("No path found.")

        elif choice == 9:
            break

        else:
            print("Invalid choice!")


if __name__ == "__main__":
    main()
