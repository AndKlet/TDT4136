from Map import Map_Obj
import heapq

class Node:
    
    """
    Constructor for Node object
    
    This class stores the position, parent node and cost for a node/cell

    __lt__ Overrides the < operator to use the total cost for nodes when comparing node1 < node2. This is used for ordering of nodes in the priority queue used in the algorithm.
    """
    def __init__(self, position, parent=None):
        self.position = position
        self.parent = parent # Parent node
        self.actual = 0  # Actual cost
        self.heuristic = 0  # Heuristic cost
        self.total = 0  # Total cost (Actual cost + Heuristic cost)

    # Override < operator behavior for priority queue
    def __lt__(self, other):
        return self.total < other.total
    


def heuristic(node, goal_node, mode=1):
    """
    Calculate the heuristic estimate from the current node to the goal node.

    Parameters:
    - node (Node): The current node
    - goal_node (Node): The goal node
    - mode (int): The heuristic calculation mode. Defaults to 1. 
        - 1: Manhattan distance
        - 2: Euclidean distance

    Returns:
    - float: The heuristic estimate from the current node to the goal node.
    """
    x = abs(node.position[0] - goal_node.position[0])
    y = abs(node.position[1] - goal_node.position[1])
    
    # Mode 1 is manhattan distance 
    if mode == 1:
        return x + y
    # Mode 2 is euclidian distance
    return x**2+y**2

def a_star(map, start, goal, heuristic_mode):
    """
    Implementation of the A* algorithm that takes a map_obj, start pos, goal pos and the heuristic mode (manhattan or euclidian)

    Parameters:
        map: map_obj
        start: tuple(x,y) of the start position
        goal: tuple(x,y) of the goal position
        heuristic_mode: 

    The function uses a priority queue to store nodes in a list, sorted by their cost.

    Returns:
        A list of coordinates representing the path from start to goal
        If no path is found it returns None

    Notes:
        Research for algorithm was done at https://saturncloud.io/blog/implementing-the-a-algorithm-in-python-a-stepbystep-guide/

    """
    # Create start and goal nodes
    start_node = Node(start)
    goal_node = Node(goal)

    # Initialize open and closed lists
    # Open list represent the nodes that are yet to be evaluated
    # Closed list represents the nodes that have already been evaluated (this is to make sure that no duplicate nodes are evaluated when checking neighbors or children)
    open_list = []
    closed_list = []

    # Add the start node to the open list
    heapq.heappush(open_list, start_node)

    # Loop until the open list is empty
    while open_list:
        # Get the node with the lowest cost from the open list
        current_node = heapq.heappop(open_list)
        
        # Appends 
        closed_list.append(tuple(current_node.position))

        # If we have reached the goal node, return the path
        if current_node.position == goal_node.position:
            return path_to_goal(current_node)

        # Generate children of the current node
        children = []
        for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
            child_position = (current_node.position[0] + dy, current_node.position[1] + dx)

            # Skips the walls (cost -1)
            if map.get_cell_value(list(child_position)) == -1:
                continue

            child_node = Node(child_position, current_node)
            # Adds child nod to children list
            children.append(child_node)

        # Iterate over children list and check whether node has already been evaluated
        for child in children:
            if tuple(child.position) in closed_list:
                continue

            # Calculate costs
            add_costs(map, child, child_position, current_node, goal_node, heuristic_mode)

            if child not in open_list:
                heapq.heappush(open_list, child)

    # If no path is found, return None
    return None

def add_costs(map, child, child_position, current_node, goal_node, heuristic_mode):
    """
    Calculate the costs for a given child node during the A* search.
    
    The function calculates the following:
        a: The actual cost from the start node to the child node.
        h: The estimated cost (heuristic) from the child node to the goal node.
        t: The total cost which is the sum of a and h.
    """
    child.actual = current_node.actual + map.get_cell_value(list(child_position))
    child.heuristic = heuristic(child, goal_node, heuristic_mode)
    child.total = child.actual + child.heuristic

def path_to_goal(node):
    """
    Constructs a path from the start node to the given node by backtracking through the parent nodes.

    Parameters:
        node: current node
    
    Returns:
        path: path from start to goal (tuples(x,y))

    """
    # Initialize an empty list to store the path
    path = []

    # Iterate backwards using parent node or each node
    while node is not None:
        path.append(node.position)
        node = node.parent

    # Return the reversed path to get the correct order
    return path[::-1]

if __name__ == "__main__":
    task = 4
    samfundet_map1 = Map_Obj(task)
    start = samfundet_map1.get_start_pos()
    goal = samfundet_map1.get_goal_pos()
    #The last parameter in a_star() is the heuristic mode (1 for manhattan, 2 for euclidian)
    path = a_star(samfundet_map1, tuple(start), tuple(goal), 1)
    if path:
        for pos in path:
            if pos == path[0] or pos == path[len(path) - 1]:
                continue
            samfundet_map1.set_cell_value(pos, ' G ')

    print(f'Task: {task}')
    print(f' Cost: {len(path)}')
    samfundet_map1.show_map()