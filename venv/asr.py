#!/usr/bin/env python
########################################################################
# History
# ------------------------------------------------
# Author     Date      		Comments
# Steves Sestari            5 Dec 19 		Initial Authoring
#
'''
## License
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/gpl-3.0.txt>.
'''


import numpy as np
import matplotlib.pyplot as plt
import random
import heapq as heap
import time
from gopigo import *  # Has the basic functions for controlling the GoPiGo Robot


def draw():
    world = {}
    for i in range(59):
        for j in range(59):
            world[(i, j)] = {'visited': False, 'dist': np.inf, 'valid': True, 'F': np.inf}
            # Obstacle A
            if i in range(29, 40) and j in range(29, 37):
                world[(i, j)]['valid'] = False
            """
             # Obstacle B
            if i in range(13, 19) and j in range(10, 17):
                world[(i, j)]['valid'] = False
            # Obstacle C
            if i in range(9, 13) and j in range(16, 21):
                world[(i, j)]['valid'] = False
            # Obstacle D
            if i in range(18, 25) and j in range(16, 20):
                world[(i, j)]['valid'] = False
            # Obstacle E
            if i in range(20, 29) and j in range(6, 20):
                if j <= 13 * i / 8 - 212 / 8:
                    world[(i, j)]['valid'] = False
            # Obstacle F
            if (i in range(12, 29) and j in range(25, 29)) or (i in range(25, 29) and j in range(22, 26)):
                world[(i, j)]['valid'] = False
            """

    return world


def isValid(coordinate, world):
    # isValid determines if coordinates are valid for the given world
    for itr in coordinate:
        # If outside the determined space
        if itr not in range(0, 59):
            return False
        for key in world.keys():
            if coordinate == key:
                if not world[key]['valid']:
                    return False
    return True


def H(coordinate, target):
    # Function H represents the Heuristic value for A*
    # The Heuristic Value = distance from a given node (or coordinate) to target
    x_dist = coordinate[0] - target[0]
    y_dist = coordinate[1] - target[1]
    diag_dist = np.sqrt((x_dist) ** 2 + (y_dist) ** 2)

    return diag_dist


def aStarAlg(world, start, end):
    # start is the start, end is at finish

    q = []

    # Direction of new path
    direct = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]

    # Heap with coordinates(x,y) and distance from the start
    heap.heappush(q, (0, start))

    world[start]['dist'] = 0
    # Where F is the total distance (distance travelled + heuristic value)
    world[start]['F'] = 0

    while q != []:
        v = heap.heappop(q)
        coordinate = v[1]

        if coordinate == end:
            break

        # Identify nodes visited
        plt.scatter(coordinate[0], coordinate[1], marker='*', color='.75')

        for direction in direct:
            new_coordinate = (direction[0] + coordinate[0], direction[1] + coordinate[1])
            if isValid(new_coordinate, world):
                if direction in [(1, 1), (-1, 1), (1, -1), (-1, -1)]:
                    dist = world[coordinate]['dist'] + np.sqrt(2)
                    F = dist + H(new_coordinate, end)
                else:
                    dist = world[coordinate]['dist'] + 1
                    F = dist + H(new_coordinate, end)
                if F < world[new_coordinate]['F']:
                    world[new_coordinate]['parent'] = coordinate
                    world[new_coordinate]['dist'] = dist
                    world[new_coordinate]['F'] = F
                if world[new_coordinate]['visited'] != True and coordinate != new_coordinate:
                    world[new_coordinate]['visited'] = True
                    heap.heappush(q, (world[new_coordinate]['F'], new_coordinate))

    # To find shortest path - backtrack through parents of nodes
    itr = end
    p = [end]
    shortest_p = world[end]['dist']

    while itr != start:
        itr = world[itr]['parent']
        p.append(itr)

    return shortest_p, p


if __name__ == '__main__':
    world = draw()

    # Create coords
    coordinates = []
    for key in world.keys():
        if world[key]['valid'] != True:
            coordinates.append(key)

    x_coord = []
    for keys in coordinates:
        x_coord.append(keys[0])

    y_coord = []
    for keys in coordinates:
        y_coord.append(keys[1])

    # Plot world
    plt.axis([0, 59, 0, 59])
    plt.scatter(x_coord, y_coord, color='red')

    # Initalize start and end points
    start = (0, 0)
    end = (58, 58)

    shortest_p, p = aStarAlg(world, start, end)
    p.reverse()

    x_coord = [itr[0] for itr in p]
    y_coord = [itr[1] for itr in p]

    # Plot path
    plt.plot(x_coord, y_coord, color='cyan')

    # Plot start and end points
    plt.plot([0], [0], marker='o', color='green')
    plt.plot([59], [59], marker='o', color='red')

    plt.title("Robot Path Planning")
    plt.show()
    print(p)
    distance_to_obj = 20  # Distance from obstacle to trigger the alert
    dire_header = -45
    for cord in range(0, len(p) - 1):
        dist = us_dist(15)  # Find the distance of the object in front
        if dist < distance_to_obj:  # If the object is closer than the "distance_to_obj" distance, trigger the alert
            print("Object Detected at Distance:", dist, 'cm')
        if dire_header == -90:
            if p[cord + 1][0] > p[cord][0]:
                if p[cord + 1][1] == p[cord][1]:
                    right()
                    time.sleep(0.7)
                    stop()
                    fwd()
                    time.sleep(0.25)
                    stop()
                    dire_header = 0
                    print("fwd+r90")
                    # print(p[cord + 1])
                elif p[cord + 1][1] < p[cord][1]:
                    right()
                    time.sleep(1)
                    stop()
                    fwd()
                    time.sleep(0.25)
                    stop()
                    dire_header = 45
                    print("fwd+r135")
                    # print(p[cord + 1])
                elif p[cord + 1][1] > p[cord][1]:
                    right()
                    time.sleep(0.4)
                    stop()
                    fwd()
                    time.sleep(0.25)
                    stop()
                    dire_header = -45
                    print("fwd+r45")
                    # print(p[cord + 1])
            if p[cord + 1][0] == p[cord][0]:
                if p[cord + 1][1] > p[cord][1]:
                    fwd()
                    time.sleep(0.25)
                    stop()
                    dire_header = -90
                    print("fwd")
                    # print(p[cord + 1])
                elif p[cord + 1][1] < p[cord][1]:
                    right()
                    time.sleep(1.1)
                    stop()
                    fwd()
                    time.sleep(0.25)
                    stop()
                    dire_header = 90
                    print("fwd+r180")
                    # print(p[cord + 1])
        elif dire_header == -45:
            if p[cord + 1][0] > p[cord][0]:
                if p[cord + 1][1] == p[cord][1]:
                    right()
                    time.sleep(0.4)
                    stop()
                    fwd()
                    time.sleep(0.25)
                    stop()
                    dire_header = 0
                    print("fwd+r45")
                    # print(p[cord + 1])
                elif p[cord + 1][1] < p[cord][1]:
                    right()
                    time.sleep(0.7)
                    stop()
                    fwd()
                    time.sleep(0.25)
                    stop()
                    dire_header = 45
                    print("fwd+r90")
                    # print(p[cord + 1])
                elif p[cord + 1][1] > p[cord][1]:
                    fwd()
                    time.sleep(0.25)
                    stop()
                    dire_header = -45
                    print("fwd")
                    # print(p[cord + 1])
            if p[cord + 1][0] == p[cord][0]:
                if p[cord + 1][1] > p[cord][1]:
                    left()
                    time.sleep(0.4)
                    stop()
                    fwd()
                    time.sleep(0.25)
                    stop()
                    dire_header = -90
                    print("fwd+l45")
                    # print(p[cord + 1])
                elif p[cord + 1][1] < p[cord][1]:
                    right()
                    time.sleep(1)
                    stop()
                    fwd()
                    time.sleep(0.25)
                    stop()
                    dire_header = 90
                    print("fwd+r135")
                    # print(p[cord + 1])
        elif dire_header == 0:
            if p[cord + 1][0] > p[cord][0]:
                if p[cord + 1][1] == p[cord][1]:
                    fwd()
                    time.sleep(0.25)
                    stop()
                    dire_header = 0
                    print("fwd")
                    # print(p[cord + 1])
                elif p[cord + 1][1] < p[cord][1]:
                    right()
                    time.sleep(0.4)
                    stop()
                    fwd()
                    time.sleep(0.25)
                    stop()
                    dire_header = 45
                    print("fwd+r45")
                    # print(p[cord + 1])
                elif p[cord + 1][1] > p[cord][1]:
                    left()
                    time.sleep(0.4)
                    stop()
                    fwd()
                    time.sleep(0.25)
                    stop()
                    dire_header = -45
                    print("fwd+l45")
                    # print(p[cord + 1])
            if p[cord + 1][0] == p[cord][0]:
                if p[cord + 1][1] > p[cord][1]:
                    left()
                    time.sleep(0.7)
                    stop()
                    fwd()
                    time.sleep(0.25)
                    stop()
                    dire_header = -90
                    print("fwd+l90")
                    # print(p[cord + 1])
                elif p[cord + 1][1] < p[cord][1]:
                    right()
                    time.sleep(0.7)
                    stop()
                    fwd()
                    time.sleep(0.25)
                    stop()
                    dire_header = 90
                    print("fwd+r90")
                    # print(p[cord + 1])
        elif dire_header == 45:
            if p[cord + 1][0] > p[cord][0]:
                if p[cord + 1][1] == p[cord][1]:
                    left()
                    time.sleep(0.4)
                    stop()
                    fwd()
                    time.sleep(0.25)
                    stop()
                    dire_header = 0
                    print("fwd+l45")
                    # print(p[cord + 1])
                elif p[cord + 1][1] < p[cord][1]:
                    fwd()
                    time.sleep(0.25)
                    stop()
                    dire_header = 45
                    print("fwd")
                    # print(p[cord + 1])
                elif p[cord + 1][1] > p[cord][1]:
                    left()
                    time.sleep(0.7)
                    stop()
                    fwd()
                    time.sleep(0.25)
                    stop()
                    dire_header = -45
                    print("fwd+l90")
                    # print(p[cord + 1])
            if p[cord + 1][0] == p[cord][0]:
                if p[cord + 1][1] > p[cord][1]:
                    left()
                    time.sleep(1)
                    stop()
                    fwd()
                    time.sleep(0.25)
                    stop()
                    dire_header = -90
                    print("fwd+l135")
                    # print(p[cord + 1])
                elif p[cord + 1][1] < p[cord][1]:
                    right()
                    time.sleep(0.4)
                    stop()
                    fwd()
                    time.sleep(0.25)
                    stop()
                    dire_header = 90
                    print("fwd+r45")
                    # print(p[cord + 1])
        elif dire_header == 90:
            if p[cord + 1][0] > p[cord][0]:
                if p[cord + 1][1] == p[cord][1]:
                    left()
                    time.sleep(0.7)
                    stop()
                    fwd()
                    time.sleep(0.25)
                    stop()
                    dire_header = 0
                    print("fwd+l90")
                    # print(p[cord + 1])
                elif p[cord + 1][1] < p[cord][1]:
                    left()
                    time.sleep(0.4)
                    stop()
                    fwd()
                    time.sleep(0.25)
                    stop()
                    dire_header = 45
                    print("fwd+l45")
                    # print(p[cord + 1])
                elif p[cord + 1][1] > p[cord][1]:
                    left()
                    time.sleep(1)
                    stop()
                    fwd()
                    time.sleep(0.25)
                    stop()
                    dire_header = -45
                    print("fwd+l135")
                    # print(p[cord + 1])
            if p[cord + 1][0] == p[cord][0]:
                if p[cord + 1][1] > p[cord][1]:
                    left()
                    time.sleep(1.1)
                    stop()
                    fwd()
                    time.sleep(0.25)
                    stop()
                    dire_header = -90
                    print("fwd+l180")
                    # print(p[cord + 1])
                elif p[cord + 1][1] < p[cord][1]:
                    fwd()
                    time.sleep(0.25)
                    stop()
                    dire_header = 90
                    print("fwd")
                    # print(p[cord + 1])
