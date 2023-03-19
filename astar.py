import numpy as np
import matplotlib.pyplot as plt
import heapq

# Dimensions of the map
width = 600
height = 250

#Clearances
obstacle_clearance = 10
robot_radius = 10

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

def plot(obstacle_map):
    plt.imshow(obstacle_map,"PuBu")
    plt.show()

obstacle_map = ObstacleMap(width, height)
plot(obstacle_map)
