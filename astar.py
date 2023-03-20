import numpy as np
import matplotlib.pyplot as plt
import heapq

# Dimensions of the map
width = 600
height = 250

#Clearances
obstacle_clearance = 10
robot_radius = 10

# Stepsize
step_size = 1

def ObstacleMap(width, height):
    map = np.full((height, width), 0)
    for y in range(height):
        for x in range(width):

            # Box obstacle
            l1 = (x-(obstacle_clearance+robot_radius)) - 150
            l2 = (x+(obstacle_clearance+robot_radius)) - 100
            l3 = y-(obstacle_clearance+robot_radius) - 100
            l4 = y+(obstacle_clearance+robot_radius) - 150

            if(l1<0 and l2>0):
                map[y, x] = 1

            if(l3 >0 and l4 < 0):
                map[y, x] = 1

            # Box Robot clearance
            l1_c = (x) - 150
            l2_c = (x) - 100
            l3_c = y - 100
            l4_c = y - 150

            if (l1_c < 0 and l2_c > 0):
                map[y, x] = 2

            if (l3 > 0 and l4 < 0):
                map[y, x] = 0

            #triangle clearance
            t1_c = (x+(obstacle_clearance+robot_radius)) - 460
            t2_c = (y) + 2 * x - 1145 - (obstacle_clearance+robot_radius)*2.2360679
            t3_c = (y) - 2 * x + 895 + (obstacle_clearance+robot_radius)*2.2360679
            if (t1_c > 0 and t2_c < 0 and t3_c > 0):
                map[y, x] = 1

            # traiangle obstacle
            t1 = (x) - 460
            t2 = (y) + 2 * (x) - 1145
            t3 = (y) - 2 * (x) + 895

            if (t1 > 0 and t2 < 0 and t3 > 0):
                map[y, x] = 2

            # Hexagon Obstacle (clearance)
            h1_c = y - 0.577 * x + 123.21 + (obstacle_clearance+robot_radius)*1.1545254
            h2_c = x - 364.95 - (obstacle_clearance+robot_radius)
            h3_c = y + 0.577 * x - 373.21 - (obstacle_clearance+robot_radius)*1.1545254
            h4_c = y - 0.577 * x - 26.92 - (obstacle_clearance+robot_radius)*1.1545254
            h5_c = x - 235 + (obstacle_clearance+robot_radius)
            h6_c = y + 0.577 * x - 223.08 + (obstacle_clearance+robot_radius)*1.1545254

            if (h2_c < 0 and h5_c > 0 and h1_c > 0 and h3_c < 0 and h4_c < 0 and h6_c > 0):
                map[y, x] = 1

            #Hexagon Obstacle
            h1 = y - 0.577 * x + 123.21
            h2 = x - 364.95
            h3 = y + 0.577 * x - 373.21
            h4 = y - 0.577 * x - 26.92
            h5 = x - 235
            h6 = y + 0.577 * x - 223.08

            if (h2 < 0 and  h5 > 0 and h1 > 0 and h3 < 0 and h4 < 0 and h6 > 0):
                map[y, x] = 2

    # Map Surrrounding Clearnce
    map[:(obstacle_clearance+robot_radius), :width] = 1
    map[height - (obstacle_clearance+robot_radius):height, :width] = 1
    map[:height, :(obstacle_clearance+robot_radius)] = 1
    map[:height, width - (obstacle_clearance+robot_radius):width] = 1

    return map

# Action Sets
# returns x , y and theta
def Actionmove_ac60(x, y, theta):
    theta = (theta + 60) % 360
    x += round(step_size * np.cos(np.radians(theta)))
    y += round(step_size * np.sin(np.radians(theta)))
    return x, y, theta

def Actionmove_ac30(x,y,theta):
    theta = (theta + 30) % 360
    x += round(step_size * np.cos(np.radians(theta)))
    y += round(step_size * np.sin(np.radians(theta)))
    return x, y, theta

def Actionmove_forward(x,y,theta):
    theta = (theta) % 360
    x += round(step_size * np.cos(np.radians(theta)))
    y += round(step_size * np.sin(np.radians(theta)))
    return x, y, theta

def Actionmove_c30(x,y,theta):
    theta = (theta - 30) % 360
    x += round(step_size * np.cos(np.radians(theta)))
    y += round(step_size * np.sin(np.radians(theta)))
    return x, y, theta

def Actionmove_c60(x,y,theta):
    theta = (theta - 60) % 360
    x += round(step_size * np.cos(np.radians(theta)))
    y += round(step_size * np.sin(np.radians(theta)))
    return x, y, theta

actions = [Actionmove_forward, Actionmove_c30, Actionmove_c60, Actionmove_ac30, Actionmove_ac60]

def compute_neighbours(map, node, actions, step_size):
    height, width = map.shape
    neighbours = []
    y, x, theta = node

    for action in actions:
        new_x, new_y, new_theta = action(x, y, theta)
        # Check if generated nodes are not an obstacle
        if 0 <= new_x < width and 0 <= new_y < height and map[new_y, new_x] == 0:
            neighbours.append((new_y, new_x, new_theta))

    return neighbours

def euclidean_distance(a, b):
    return np.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

def A_star(start, goal, map, actions, step_size):
    open_list = PriorityQueue()
    open_list.put((0, start))
    cost_to_come = {start: 0}
    parent = {start: None}
    closed_list = set()
    all_nodes = []

    while not open_list.empty():

        current_cost, current_node = open_list.get()

        # checking if robot reached the goal
        if euclidean_distance(current_node, goal) < 1.5 and goal[2] == current_node[2]:
                current_node == goal
                print('Goal reached!')
                # finding the optimal path
                optimal_path = []
                while current_node is not None:
                    optimal_path.append(current_node)
                    current_node = parent[current_node]
                optimal_path.reverse()
                return all_nodes, optimal_path

        closed_list.add(current_node)

        # computing valid neighbours based on actions
        neighbors = compute_neighbours(map, current_node, actions, step_size)

        for neighbor in neighbors:

            # calculating cost to come to the neighbor
            cost_to_come_temp = cost_to_come[current_node]

            # checking if it's in closed list
            if neighbor in closed_list:
                continue

            # checking if it's in open list with best cost
            if neighbor in cost_to_come and cost_to_come_temp >= cost_to_come[neighbor]:
                continue

            # adding it to open list with total cost
            cost_to_come[neighbor] = cost_to_come_temp
            total_cost = cost_to_come_temp + euclidean_distance(neighbor, goal)
            open_list.put((total_cost, neighbor))
            parent[neighbor] = current_node

            all_nodes.append(neighbor)

    # return none when goal is not reachable
    print('Goal not reachable!')
    return None

def plot(obstacle_map):
    plt.imshow(obstacle_map,"PuBu")
    plt.show()

obstacle_map = ObstacleMap(width, height)
plot(obstacle_map)
