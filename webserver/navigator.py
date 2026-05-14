import heapq

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

map_grid = []
map_row_size = 0
map_col_size = 0

def heuristic(row_curr, col_curr, row_goal, col_goal):
    return abs(row_goal - row_curr) + abs(col_goal - col_curr)


def can_travel(row, col):
    if row < 0 or col < 0 or row >= map_row_size or col >= map_col_size:
        return False
    return map_grid[row * map_col_size + col] != 'x'


def get_neighbors(row_curr, col_curr):
    neighbors = []
    #cycle through going up left right down
    #add cycle through going upleft upright downleft downright
    for changeRow, changeCol in [(-1,0),(0,-1),(0,1),(1,0),(-1,-1),(-1,1),(1,-1),(1,1)]:
        row, col = row_curr + changeRow, col_curr + changeCol
        if can_travel(row, col):
            neighbors.append((row, col))
    return neighbors


def astar(row_start, col_start, row_end, col_end):
    if row_start == row_end and col_start == col_end:
        print("!!!!!!!! Already at goal, no A* needed !!!!!!!!")
        return None

    # node_track: (row,col) -> {'parent': (prow,pcol) or None, 'g': int, 'state': 'open'|'closed'}
    node_track = {}

    # open set: min-heap of (f, g, row, col)
    # g is stored separately so ties on f can be broken and we always have the best g
    open_set = []
    gone_start = 0
    heur_start = heuristic(row_start, col_start, row_end, col_end)
    heapq.heappush(open_set, (gone_start + heur_start, gone_start, row_start, col_start))
    node_track[(row_start, col_start)] = {'parent': None, 'g': 0, 'state': 'open'}

    while open_set:
        #distTraveled from start
        totdist, distTravel, row, col = heapq.heappop(open_set)

        info = node_track.get((row, col))
        # skip stale heap entries (a better path was already found)
        if info and info['state'] == 'closed':
            continue
        if info and info['g'] < distTravel:
            continue

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
            if neighbor_info and neighbor_info['g'] <= new_gone:
                continue

            new_h = heuristic(neighborR, neighborC, row_end, col_end)
            node_track[(neighborR, neighborC)] = {'parent': (row, col), 'g': new_gone, 'state': 'open'}
            heapq.heappush(open_set, (new_gone + new_h, new_gone, neighborR, neighborC))

    return None  # no path found


def reconstruct_path(row_end, col_end, node_track):
    path = []
    curr = (row_end, col_end)
    while curr is not None:
        path.append(curr)
        curr = node_track[curr]['parent']
    return path  # reverse order: goal -> start, matches original C behavior


roomNum = 0
def path_Astar(roomNum):
    waypoints = []
    row_end = roomNum / 10
    col_end = roomNum / 10
    print(row_end)
    waypoints = astar(0, 0, row_end, col_end)
    #return waypoints as 
    if(waypoints == None):
        return [(500,300),(600,300),(100,200),(300,100),(300,200),(700,800)]
    return waypoints.reverse()


def print_map(rows, cols):
    for r in range(rows):
        print(' '.join(map_grid[r * cols:(r + 1) * cols]))


def apply_path(path, cols):
    for (r, c) in path:
        map_grid[r * cols + c] = '-'

def main():
    global map_grid, map_row_size, map_col_size

    while True:
        print("------------------------------")
        print("Menu:")
        print("8 - solve big maze")
        print("9 - exit")
        print("------------------------------")

        choice = int(input())

        if choice == 8:
            print("Solve big maze")
            map_row_size, map_col_size = 16, 11
            map_grid = list(maze)
            print_map(16, 11)
            path = astar(0, 0, 15, 10)
            if path:
                print(f"pathLen {len(path)}")
                apply_path(path, 11)
            print_map(16, 11)

        elif choice == 9:
            break

        else:
            print("Invalid choice!")
            break


if __name__ == "__main__":
    main()
