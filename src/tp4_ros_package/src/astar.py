import heapq

class Node:
    def __init__(self, x, y, parent=None):
        self.x = x
        self.y = y
        self.parent = parent
        self.g = 0
        self.h = 0
    
    def __lt__(self, other):
        return (self.g + self.h) < (other.g + other.h)

def heuristic(node, goal):
    return abs(node.x - goal.x) + abs(node.y - goal.y)

def astar(grid, start, goal):
    open_list = []
    closed_set = set()

    heapq.heappush(open_list, start)

    while open_list:
        current_node = heapq.heappop(open_list)

        if current_node.x == goal.x and current_node.y == goal.y:
            path = []
            while current_node:
                path.append((current_node.x, current_node.y))
                current_node = current_node.parent
            return path[::-1]

        closed_set.add((current_node.x, current_node.y))

        for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            next_x, next_y = current_node.x + dx, current_node.y + dy

            if next_x < 0 or next_x >= len(grid) or next_y < 0 or next_y >= len(grid[0]) or grid[next_x][next_y] == 1:
                continue

            if (next_x, next_y) in closed_set:
                continue

            new_g = current_node.g + 1
            new_h = heuristic(Node(next_x, next_y), goal)

            next_node = Node(next_x, next_y, current_node)
            next_node.g = new_g
            next_node.h = new_h

            heapq.heappush(open_list, next_node)

    return None

# Example usage
grid = [
    [0, 0, 0, 0, 0],
    [0, 1, 1, 1, 0],
    [0, 1, 0, 1, 0],
    [0, 0, 0, 0, 0]
]

start_node = Node(0, 0)
goal_node = Node(3, 4)

path = astar(grid, start_node, goal_node)
if path:
    print("Path found:", path)
else:
    print("No path found")
