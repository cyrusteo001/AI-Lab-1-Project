import json
import math as m

# E_CONSTRAINT = 1000000
E_CONSTRAINT = 287932

with open('jsonFiles/Coord.json') as coord:
    coord_data = json.load(coord)
with open('jsonFiles/Cost.json') as cost:
    cost_data = json.load(cost)
with open('jsonFiles/Dist.json') as dist:
    dist_data = json.load(dist)
with open('jsonFiles/G.json') as g:
    g_data = json.load(g)

''' To solve task 1 - using Uniform cost search without constraint '''


# To get the path, in the visited dictionary, key = current node and value = parent node. Then work backwards to find path taken
def uniform_CS(start_node, end_node):
    # Set answer to a very large number first
    answer = 10 ** 10
    # Utilise priority queue
    queue = []
    # insert the starting index. (dist, node, parent_node, energy_consumption)
    # Dist from start node to start node is 0
    queue.append([0, str(start_node), str(-1), 0])
    # map to store visited nodes
    visited = {}

    # while the queue is not empty
    while len(queue) > 0:
        # get the top element of the queue. Sort by cost queue[0] and pop the one with the smallest cost
        queue = sorted(queue)
        p = queue.pop(0)

        # check if the element is goal node
        if int(p[1]) == end_node:
            # if the cost is less than current answer, update the answer
            if answer > p[0]:
                answer = p[0]
                # To speed things up, check whether the cost of this node is less than all other nodes in the queue
                # If yes, we can immediately return answer and not check the others
                j = 0
                while j < (len(queue) - 1):
                    if p[0] > queue[j][0]:
                        break
                    j += 1
                if j == (len(queue) - 1):
                    visited[p[1]] = p[2]
                    path = [p[1]]
                    current = p[1]
                    while current != '1':
                        current = visited[current]
                        path.append(current)
                    return answer, path, p[3]

        # Expand adjacent nodes. Make sure nodes expanded not in visited list
        for i in g_data[str(p[1])]:
            if str(i) not in visited:
                # append expanded and unvisited nodes into queue, including their dist + the cost of current node
                try:
                    distance = dist_data[str(p[1]) + ',' + str(i)]
                except:
                    distance = dist_data[str(i) + ',' + str(p[1])]

                try:
                    energy = cost_data[str(p[1]) + ',' + str(i)]
                except:
                    energy = cost_data[str(i) + ',' + str(p[1])]

                queue.append([(p[0] + distance), str(i), p[1], (p[3] + energy)])

        # mark current node as visited
        visited[p[1]] = p[2]
    # if no other node in the queue, no need to check any further. return answer
    # But if answer is still original value, return no -1
    if answer == 10 ** 10:
        return -1
    visited[p[1]] = p[2]
    path = [p[1]]
    current = p[1]
    while current != '1':
        current = visited[current]
        path.append(current)
    return answer, path, p[3]


''' To solve task 2 - using Uniform cost search with constraint'''


# Reuse UCS from task 1 but with added energy constraint
def uniform_CS_with_constraint(start_node, end_node, constraint):
    # Set answer to a very large number first
    answer = 10 ** 10
    # Utilise priority queue
    queue = []
    # insert the starting index. (dist, node, energy_consumption, parent_node)
    # Dist from start node to start node is 0
    queue.append([0, str(start_node), 0, -1])
    # map to store visited nodes
    visited = {}

    # while the queue is not empty
    while len(queue) > 0:
        # get the top element of the queue. Sort by cost queue[0] and pop the one with the smallest cost
        queue = sorted(queue)
        p = queue.pop(0)
        # Check if the energy consumption at this node is already exceeded:
        if p[2] < constraint:
            # check if the element is goal node
            if int(p[1]) == end_node:
                # if the cost is less than current answer
                if answer > p[0]:
                    answer = p[0]
                    # To speed things up, check whether the cost of this node is less than all other nodes in the queue
                    # If yes, we can immediately return answer and not check the others
                    j = 0
                    while j < (len(queue) - 1):
                        if p[0] > queue[j][0]:
                            break
                        j += 1
                    if j == (len(queue) - 1):
                        visited[p[1]] = p[3]
                        path = [p[1]]
                        current = p[1]
                        while current != '1':
                            current = visited[current]
                            path.append(current)
                        return answer, p[2], path

            # Expand adjacent nodes. Make sure nodes expanded not in visited list
            for i in g_data[str(p[1])]:
                if str(i) not in visited:
                    # append expanded and unvisited nodes into queue, including their dist + the cost of current node, and current energy consumption
                    try:
                        distance = dist_data[str(p[1]) + ',' + str(i)]
                    except:
                        distance = dist_data[str(i) + ',' + str(p[1])]

                    try:
                        energy = cost_data[str(p[1]) + ',' + str(i)]
                    except:
                        energy = cost_data[str(i) + ',' + str(p[1])]

                    queue.append([(p[0] + distance), str(i), (p[2] + energy), p[1]])

            # only mark current node as visited if it was considered within energy constraint
            visited[p[1]] = p[3]
    # if no other node in the queue, no need to check any further. return answer
    # But if answer is still original value, return no -1
    if answer == 10 ** 10:
        # If no viable answer return -1
        return -1, -1, -1
    visited[p[1]] = p[3]
    path = [p[1]]
    current = p[1]
    while current != '1':
        current = visited[current]
        path.append(current)
    return answer, p[2], path


''' To solve task 3 - Using A* search algorithm '''


# Create a function to find euclidean distances
def find_euclidean_distance(node_1, node_2):
    node1coor = coord_data[str(node_1)]
    node2coor = coord_data[str(node_2)]
    return m.sqrt(((node1coor[0] - node2coor[0]) ** 2) + ((node1coor[1] - node2coor[1]) ** 2))


# Reuse UCS from task 2, but with added heuristic of straight line distance of evaluated points
def a_star_search(start_node, end_node, constraint):
    # Set answer to a very large number first
    answer = 10 ** 10
    # Utilise priority queue
    queue = []
    # insert the starting index. (evaluation value, node, energy_consumption, distance_covered_so_far, parent_node)
    # Dist from start node to start node is 0
    # Find dist from start node to end node using euclidean distance function
    queue.append([0 + find_euclidean_distance(1, 50), str(start_node), 0, 0, -1])
    # map to store visited nodes
    visited = {}

    # while the queue is not empty
    while len(queue) > 0:
        # get the top element of the queue. Sort by cost queue[0] and pop the one with the smallest cost
        queue = sorted(queue)
        p = queue.pop(0)
        # Check if the energy consumption at this node is already exceeded:
        if p[2] < constraint:
            # check if the element is goal node
            if int(p[1]) == end_node:
                # if the cost is less than current answer
                if answer > p[3]:
                    answer = p[3]
                    # To speed things up, check whether the evaluation cost of this node is less than all other nodes
                    # in the queue. If yes, we can immediately return answer and not check the others
                    j = 0
                    while j < (len(queue) - 1):
                        if p[0] > queue[j][0]:
                            break
                        j += 1
                    if j == (len(queue) - 1):
                        visited[p[1]] = p[4]
                        path = [p[1]]
                        current = p[1]
                        while current != '1':
                            current = visited[current]
                            path.append(current)
                        return answer, p[2], path

            # Expand adjacent nodes. Make sure nodes expanded not in visited list
            for i in g_data[str(p[1])]:
                if str(i) not in visited:
                    # append expanded and unvisited nodes into queue, including the distance to the node + the node's euclidean distance from goal_node, and current energy consumption
                    try:
                        distance = dist_data[str(p[1]) + ',' + str(i)]
                    except:
                        distance = dist_data[str(i) + ',' + str(p[1])]

                    try:
                        energy = cost_data[str(p[1]) + ',' + str(i)]
                    except:
                        energy = cost_data[str(i) + ',' + str(p[1])]

                    queue.append([(find_euclidean_distance(i, p[1]) + distance), str(i), (p[2] + energy), (p[3] + distance), p[1]])

            # mark current node as visited
            visited[p[1]] = p[4]
    # if no other node in the queue, no need to check any further. return answer
    # But if answer is still original value, return no -1
    if answer == 10 ** 10:
        # If no viable answer return -1
        return -1, -1, -1
    visited[p[1]] = p[4]
    path = [p[1]]
    current = p[1]
    while current != '1':
        current = visited[current]
        path.append(current)
    return answer, p[2], path


task1_dist, task1_path, task1_energy = uniform_CS(1, 50)
path1 = ""
path1 = path1+task1_path.pop(-1)
for i in reversed(task1_path):
    path1 = path1+"->"+i
print("Beginning task 1...")
print("Shortest path: " + path1)
print("Shortest distance: " + str(task1_dist))
print("Total Energy Cost: " + str(task1_energy))

task2_dist, task2_energy, task2_path = uniform_CS_with_constraint(1, 50, E_CONSTRAINT)
print("")
print("Beginning task 2...")
if task2_dist == -1:
    print("This is impossible as energy cost exceeded for all possible paths")
else:
    path2 = ""
    path2 = path2 + task2_path.pop(-1)
    for i in reversed(task2_path):
        path2 = path2 + "->" + i
    print("Shortest path: " + path2)
    print("Shortest distance: " + str(task2_dist))
    print("Total Energy Cost: " + str(task2_energy))

task3_dist, task3_energy, task3_path = a_star_search(1, 50, E_CONSTRAINT)
print("")
print("Beginning task 3...")
if task3_dist == -1:
    print("This is impossible as energy cost exceeded for all possible paths")
else:
    path3 = ""
    path3 = path3 + task3_path.pop(-1)
    for i in reversed(task3_path):
        path3 = path3 + "->" + i
    print("Shortest path: " + path3)
    print("Shortest distance: " + str(task3_dist))
    print("Total Energy Cost: " + str(task3_energy))
