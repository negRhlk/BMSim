import random
import networkx as nx
from detect_neighbor import detect_neighbor
from node import node
import matplotlib.pyplot as plt
import math
import numpy as np
from pylab import plot, show, savefig, xlim, figure, \
    ylim, legend, boxplot, setp, axes

Gar = nx.Graph()


def position_finder(NUMBER_NODES, ENVIRONMENT, NODE_RANGE, Gar):
    x = []
    y = []
    for i in range(NUMBER_NODES // 2):
        xp = random.randint(1, ENVIRONMENT // 2 + 20)
        x.append(xp)
        yp = random.randint(1, ENVIRONMENT // 2 + 20)
        y.append(yp)

    for i in range(NUMBER_NODES - NUMBER_NODES // 2):
        xp = random.randint(ENVIRONMENT // 2 - 20, ENVIRONMENT)
        x.append(xp)
        yp = random.randint(ENVIRONMENT // 2 - 20, ENVIRONMENT)
        y.append(yp)

    nodes = []

    # nodes are randomly distributed in the environment
    for i1 in range(NUMBER_NODES):
        nodes.append(node(i1, x[i1], y[i1]))
        Gar.add_node(nodes[i1].ID, pos=(nodes[i1].Xposition, nodes[i1].Yposition))

    # ---------------------------- Detect Neighbour ---------------------------
    for node_source in range(NUMBER_NODES):  # neighbours is determined
        neighbour = detect_neighbor(node_source, NODE_RANGE, NUMBER_NODES, nodes, Gar)
        nodes[node_source].neighbours = neighbour

    degree = set()
    for i in range(NUMBER_NODES):
        degree.add(len([n for n in Gar.neighbors(i)]))
        # print(degree)

    if nx.is_connected(Gar):
        #          ----------------------- Plot Network Topology ----------------------
        fig = figure()
        pos = {}
        for i in range(len(Gar.nodes)):
            pos[i] = np.array([nodes[i].Xposition, nodes[i].Yposition])
        nx.draw(Gar, pos=pos, with_labels=True)
        plt.show()
        #          ----------------------------- Returns ------------------------------

        for i in range(NUMBER_NODES):
            print("node", i, ' : ', nodes[i].neighbours)
            print("node", i, ' : ', Gar.degree[i])
            print("node", i, ' : ', [n for n in Gar.neighbors(i)])
        return x, y
    else:
        x = []
        y = []
        return x, y


x = []
y = []
nodes_num = 200
env = 120
node_range = 11.26

while x == [] and y == []:
    x, y = position_finder(nodes_num, env, node_range, Gar)
    Gar.clear()
    Gar = nx.Graph()
print('x =', x)
print('y =', y)
