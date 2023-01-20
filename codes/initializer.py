"""
The setting of the network is determined in this file.
For plugging in static models and using static algorithms in the network design-time you can use the initializer.py file
"""

import random
import networkx as nx
import numpy as np
from node import node
import pymobility
from pymobility.models.mobility import random_waypoint
from logger import setup_logger
import logging
from detect_neighbor import detect_neighbor
from choice_feature import choice_feature
import matplotlib.pyplot as plt
import math
from pylab import plot, show, savefig, xlim, figure, \
    ylim, legend, boxplot, setp, axes
from x_y_finder import position_finder

# ---------------------------- global variables -------------------------------
global update_mobility_interval, lowpower_Poll_interval
global BUFFER_SIZE, reception_ratio
global NUMBER_NODES, ENVIRONMENT, NUMBER_RELAY_NODE, NUMBER_RELAY_G_NODE
global heartbeat_log, logger, energy_log, mobility_flag, R_h, x, y, next_update, Relay_Node, Relay_G_Node
global NETWORK_TTL, PACKET_LENGTH, EXECUTION_TIME, TOTAL_LOG, BYTE, DATA_RATE, SCAN_STEP, SWITCH_TIME
global Advertise_time, Receive_Delay, sleep_time, Receive_window, destination, max_seq
global GENERATION_EVENT_Adv37, HEARTBEAT_EVENT_Adv37, RELAY_EVENT_Adv37, AD38_EVENT, AD39_EVENT, AD39_EVENT_End
global SCAN37_EVENT, SCAN38_EVENT, SCAN39_EVENT, SCAN37_C_EVENT, SCAN38_C_EVENT, SCAN39_C_EVENT, SWITCH_37TO38
global SWITCH_38TO39, SEND_POLL, FRIEND_RELAY, NODE_TIME, NODE_EVENT, Time, i_node
global SINK_NODE, JUST_RELAY, JUST_GENERATION, RELAY_AND_GENERATION, LOW_POWER, FRIEND_RELAY_NODE, FRIEND_NODE
global nodes, Center_node, Center_node_static, Show_progress_interval, Show_progress

# For logging collisions
global collisions37, collisions38, collisions39, collision_log


# -----------------------------------------------------------------------------
# ----------------------------- initial settings -------------------------------
# --------------------------- deployment setting ------------------------------
NUMBER_NODES = 10  # the number of nodes in the network
x = []  # the positions of nodes in the network
y = []

# The size of the environment that the network nodes are spread in it, 
# in this case, is 25*25 meters
ENVIRONMENT = 120

# -----------------------------------------------------------------------------
# Number of nodes with relay feature in the network
NUMBER_RELAY_NODE = int(1 / 3 * NUMBER_NODES)
print(NUMBER_RELAY_NODE)

# Number of nodes with relay and generator features in the network
NUMBER_RELAY_G_NODE = int(1 / 6 * NUMBER_NODES)
print(NUMBER_RELAY_G_NODE)
# The radius of the node communication range
NODE_RANGE = 11.26

# The execution time of the simulator in milliseconds, in this case, is 10 min
EXECUTION_TIME = 300000

destination = []
max_seq = []
Gar = nx.Graph()
Relay_Node = []
Relay_G_Node = []

# -----------------------------------------------------------------------------
random_network = 0

# Random topology
if random_network == 1:
    # x, y = position_finder(NUMBER_NODES, ENVIRONMENT, NODE_RANGE, Gar)

    # x = [16, 22, 2, 3, 12, 7, 19, 1, 23, 17, 4, 7, 21, 3, 20, 13, 15, 16, 5, 8, 6, 22, 5, 11, 18, 20, 6, 2, 5, 1, 5, 4,
    #      23, 6, 19, 5, 4, 6, 10, 6, 18, 2, 2, 5, 16, 25, 9, 14, 13, 8, 3, 5, 1, 10, 22, 3, 8, 9, 22, 5, 14, 25, 12, 1,
    #      4, 3, 1, 19, 22, 22, 19, 9, 22, 25, 25, 14, 11, 6, 9, 12, 23, 4, 9, 21, 1, 20, 21, 10, 6, 10, 14, 6, 18, 23, 5,
    #      12, 15, 12, 9, 2]
    # y = [7, 24, 21, 18, 14, 24, 17, 20, 5, 20, 12, 12, 15, 12, 1, 1, 20, 8, 15, 3, 25, 8, 12, 5, 9, 15, 23, 1, 24, 11,
    #      4, 23, 13, 13, 4, 24, 13, 16, 23, 16, 6, 24, 6, 14, 18, 21, 6, 22, 12, 6, 5, 21, 3, 7, 19, 19, 13, 8, 13, 6, 7,
    #      20, 22, 3, 24, 10, 7, 5, 2, 4, 16, 1, 13, 16, 13, 12, 12, 15, 21, 3, 16, 19, 2, 4, 6, 21, 4, 2, 16, 22, 12, 17,
    #      3, 22, 19, 5, 10, 21, 7, 16]

    # 200 nodes in 120 * 120 environment
    x = [17, 9, 59, 22, 62, 16, 62, 29, 55, 79, 22, 33, 41, 1, 79, 48, 65, 33, 66, 72, 37, 14, 59, 3, 70, 20, 73, 21,
         72, 15, 20, 7, 52, 25, 45, 72, 8, 71, 80, 9, 79, 47, 49, 3, 28, 64, 32, 19, 45, 16, 15, 44, 58, 78, 3, 70, 66,
         47, 63, 29, 12, 72, 66, 54, 35, 76, 35, 21, 72, 7, 9, 73, 45, 3, 22, 36, 29, 68, 12, 80, 43, 26, 33, 19, 50,
         21, 31, 21, 26, 16, 41, 38, 77, 79, 78, 36, 68, 62, 48, 17, 62, 47, 62, 72, 102, 87, 41, 53, 82, 96, 78, 115,
         40, 57, 94, 89, 45, 73, 115, 62, 88, 100, 46, 98, 62, 64, 81, 57, 116, 70, 80, 43, 67, 49, 90, 105, 58, 58, 90,
         93, 48, 106, 104, 66, 44, 89, 45, 60, 44, 44, 83, 100, 118, 89, 101, 71, 46, 53, 63, 51, 78, 54, 106, 109, 62,
         68, 51, 95, 89, 107, 42, 114, 114, 81, 60, 65, 109, 78, 110, 120, 107, 92, 85, 48, 73, 72, 55, 100, 75, 105,
         90, 113, 113, 112, 85, 68, 45, 89, 87, 90]
    y = [31, 9, 30, 52, 58, 20, 63, 15, 70, 40, 37, 53, 47, 76, 4, 44, 73, 58, 7, 27, 12, 6, 71, 37, 52, 25, 43, 4, 4,
         50, 51, 64, 25, 48, 30, 33, 4, 3, 67, 60, 54, 13, 51, 75, 23, 79, 8, 46, 41, 59, 2, 43, 41, 70, 11, 7, 79, 28,
         46, 46, 78, 63, 20, 21, 26, 38, 35, 24, 6, 30, 19, 16, 55, 59, 6, 22, 53, 74, 69, 21, 31, 38, 12, 5, 23, 67,
         53, 39, 55, 19, 21, 56, 50, 38, 2, 19, 27, 77, 8, 78, 46, 70, 50, 51, 94, 85, 91, 58, 83, 45, 53, 108, 87, 55,
         107, 42, 51, 102, 116, 117, 100, 104, 43, 72, 86, 43, 101, 87, 65, 112, 90, 114, 53, 113, 78, 109, 95, 66, 62,
         90, 65, 110, 99, 120, 71, 55, 93, 106, 41, 82, 52, 56, 100, 91, 97, 80, 101, 112, 72, 54, 62, 73, 59, 112, 91,
         40, 99, 101, 107, 65, 95, 47, 70, 110, 43, 83, 63, 65, 89, 93, 115, 56, 88, 115, 120, 113, 94, 58, 76, 119, 76,
         79, 75, 57, 74, 87, 47, 90, 99, 40]



# Grid topology
else:
    for i17 in range(
            int(NUMBER_NODES / math.sqrt(NUMBER_NODES)) + 5):
        for j17 in range(int(NUMBER_NODES / math.sqrt(NUMBER_NODES))):
            # x = random.randint(1,ENVIRONMENT)
            x.append(9 * i17)
            # y = random.randint(1,ENVIRONMENT)
            y.append(9 * j17)

# ---------------------------------- Logging ----------------------------------

# When TOTAL_LOG is one, each node has a separate log file,
# with detailed data about its operations during network running.

# When TOTAL_LOG is zero, there is one log file for all network nodes, and
# in this file, the data needed for calculating performance metrics are logged
TOTAL_LOG = 1

# When the amount of this variable is one, some information in detailed log 
# files in each node about the heartbeat messages is logged
heartbeat_log = 0

logger = []
energy_log = []

# ----------------------------- For Logging Collisions ----------------------------
# Flag for logging number of collisions
collision_log = 1

collisions37 = [0 for i in range(NUMBER_NODES)]
collisions38 = [0 for i in range(NUMBER_NODES)]
collisions39 = [0 for i in range(NUMBER_NODES)]

# ------------------------------- Network Setting -----------------------------
R_h = 2  # used in determining TTL by heartbeat message, this variable is added to min hop for determining the TTL  
mobility_flag = 0  # mobility flag determines the number of mobility updates during the simulation

# update flag determines the presence of run-time adjustment for parameters and models during the simulation
update_flag = 0
BUFFER_SIZE = 6  # the size of the nodes' buffer

# after this interval, the simulator shows the percentage of simulation progress
Show_progress_interval = EXECUTION_TIME / 10
update_mobility_interval = 1000  # after this interval, the simulator calls the Network_updator module 

# the initial value for the network's TTL if the user does not want to use the heartbeat message
NETWORK_TTL = NUMBER_NODES
Show_progress = 0
Show_progress = 0 + Show_progress_interval
next_update = 0
next_update = 0 + update_mobility_interval

# ----------------------------- Low power and Friend ---------------------------

# The delay period for the friend node to prepare the message
Receive_Delay = 10

sleep_time = 5  # the sleep time in friendship mechanism
Receive_window = 98  # the Receive_window parameter in friendship mechanism
lowpower_Poll_interval = 4000  # the request[friend poll] interval parameter in friendship mechanism

# ------------------------------ Node Initializing ----------------------------
nodes = []

for i1 in range(NUMBER_NODES):
    # the positions of the nodes adds to them
    nodes.append(node(i1, x[i1], y[i1]))
    # the positions of the nodes add to the network topology
    Gar.add_node(nodes[i1].ID, pos=(nodes[i1].Xposition, nodes[i1].Yposition))

for i_r in range(NUMBER_NODES):
    # ----------------------------- node setting ------------------------------
    nodes[i_r].SCAN_INTERVAL = 30
    nodes[i_r].SCAN_WINDOW = 30
    nodes[i_r].Relay_Retransmit_Count = 0
    nodes[i_r].Network_Transmit_Count = 0
    nodes[i_r].Rris = 1
    nodes[i_r].Ntis = 1
    nodes[i_r].Advertise_Interval = 20
    nodes[i_r].GENERATION_INTERVAL = 1000  # ms
    nodes[i_r].Relay_Retransmission_Interval = (nodes[i_r].Rris + 1) * 10 + random.randint(1, 10)
    nodes[i_r].Transmission_Interval = (nodes[i_r].Ntis + 1) * 10 + random.randint(1, 10)

    # ---------------------------- Initial value ------------------------------
    nodes[i_r].cache = []
    nodes[i_r].buffer = []
    nodes[i_r].keynet = []
    nodes[i_r].channel37 = 0
    nodes[i_r].channel38 = 0
    nodes[i_r].channel39 = 0
    nodes[i_r].advertisetag37 = 0  # these flags are used for collisions detection
    nodes[i_r].advertisetag38 = 0
    nodes[i_r].advertisetag39 = 0
    nodes[i_r].message = 0
    nodes[i_r].L_scan = 8  # the last scanning channel for determining the next scanning channel 
    nodes[i_r].first_time_scan = 0  # the beginning of the scan window is saved in each node
    nodes[i_r].seq_number = 0
    nodes[i_r].h_seq_number = 0
    nodes[i_r].Gen_cache = []
    nodes[i_r].heart_cache = []
    nodes[i_r].Sleep_Time = 0  # the node sleeping time
    nodes[i_r].Scan_Time = 0  # the node scanning time
    nodes[i_r].low_power_ack = 0
    nodes[i_r].Switch_Time = 0  # the node switching time
    nodes[i_r].Transmit_Time = 0  # the node transmission time 
    nodes[i_r].node_TTL = 127  # the initial value for the nodes' TTL
    nodes[i_r].n_count = 0  # this variable is used for counting the retransmissions in the generator nodes
    nodes[i_r].r_count = 0  # this variable is used for counting the retransmissions in the relay nodes 
    nodes[i_r].minhop = 127
    nodes[i_r].maxhop = 0
    nodes[i_r].Sleep_Time = 0
    nodes[i_r].init_time = 0

    # -------------------------- Initial random value -------------------------
    ''' 
    It is necessary to save the last time of executing each event, 
    for calculating the next time of executing this event 
    '''

    # the last time that the retransmission event is executed  in the relay nodes
    nodes[i_r].last_T_relay = random.randint(0, nodes[i_r].Relay_Retransmission_Interval)

    # the last time that the retransmission event is executed  in the generator nodes
    nodes[i_r].last_T_generation = random.randint(0, nodes[i_r].Transmission_Interval)

    # the last time that advertising event is executed  in the nodes
    nodes[i_r].last_relay_time = random.randint(0, nodes[i_r].Advertise_Interval)

    # the last time that scanning event is executed  in the nodes
    nodes[i_r].last_t_scan = random.randint(0, nodes[i_r].SCAN_INTERVAL)

    # the last time that sending request[friend poll] event is executed in the low-power nodes
    nodes[i_r].last_poll_time = random.randint(0, lowpower_Poll_interval)

    # the last time that the heartbeat event is executed in the nodes
    nodes[i_r].time_heartbeat = random.randint(0, 1000)

    nodes[i_r].last_seq_number = np.full(NUMBER_NODES, 0)
    nodes[i_r].h_last_seq_number = np.full(NUMBER_NODES, 0)

for i1 in range(NUMBER_NODES):
    # the last time that generation event is executed  in the generator nodes
    nodes[i1].last_generation_time = random.randint(0, nodes[i1].GENERATION_INTERVAL)

# ----------------------------- Statice Algorithms ----------------------------
# ------------------------------ Detect Neighbour -----------------------------
# by calling the detect_neighbour function the neighbors of each node is determined
for node_source in range(NUMBER_NODES):
    neighbor = detect_neighbor(node_source, NODE_RANGE, NUMBER_NODES, nodes, Gar)
    nodes[node_source].neighbors = neighbor

# ------ static algorithms for determining center node and relay nodes --------

# Closeness centrality characteristic in the network topology is used
# for choosing the sink node 
closeness = nx.closeness_centrality(Gar)
Sink = sorted(closeness.items(), key=lambda n: n[1], reverse=True)[:1]
Center_node = Sink[0][0]
Center_node_static = Center_node

# Betweenness centrality characteristic in the network topology is used
# for choosing the relay nodes
Betweenness = nx.betweenness_centrality(Gar)
best_Relay = sorted(Betweenness.items(), key=lambda n: n[1], reverse=True)[
             :NUMBER_RELAY_NODE]
best_G_Relay = sorted(Betweenness.items(), key=lambda n: n[1], reverse=True)[
               NUMBER_RELAY_NODE:NUMBER_RELAY_G_NODE + NUMBER_RELAY_NODE]

# while True:
#     selection = random.randint(0, NUMBER_NODES)
#     if selection == Center_node:
#         continue
#     elif len(Relay_Node) == NUMBER_RELAY_NODE:
#         break
#     else:
#         Relay_Node.append(selection)

# vital_closeness = nx.closeness_vitality(Gar)
#
# vital_closeness = sorted(vital_closeness.items(), key=lambda x: x[1], reverse=True)
# sorted_vital_closeness = dict(vital_closeness)
#

for j1 in range(NUMBER_RELAY_NODE):
    Relay_Node.append(best_Relay[j1][0])

# Relay_Node = [138, 115, 17, 54, 23, 98, 152, 117, 131, 79, 101, 123, 13, 171, 177, 199, 66, 69, 73, 2, 179, 184, 133,
#               62, 112, 178, 85, 109, 190, 65, 174, 27, 10, 46, 118, 143, 147, 18, 149, 191, 99, 151, 130, 145, 91, 89,
#               39, 41, 154, 173, 183, 96, 144, 192, 43, 162, 134, 35, 159, 5, 29, 63, 189, 119, 156, 14]
for j1 in range(NUMBER_RELAY_G_NODE):
    Relay_G_Node.append(best_G_Relay[j1][0])

# ---------------------------- Choice Feature ---------------------------------
# by calling the choice_feature function the feature of each node is determined
for i_f in range(NUMBER_NODES):
    choice_feature(nodes, i_f, Center_node, Relay_Node, Relay_G_Node)

# --------------------------- Heartbeat Period --------------------------------
for i_heart in range(NUMBER_NODES):
    nodes[i_heart].Heartbeat_period = 0
nodes[Center_node].Heartbeat_period = 4000

# --------------- Choosing A Static Model for Reception Ratio -----------------
reception_ratio = [[0 for x in range(NUMBER_NODES)] for y in range(NUMBER_NODES)]

# reception_ratio_gen = [44, 50, 44, 77, 58, 82, 74, 61, 51, 63, 43, 44, 42, 56, 65, 55, 45, 86, 85, 80, 77, 62, 82, 64,
#                        50, 84, 53, 71, 75, 63, 57, 84, 63, 47, 67, 82, 66, 71, 41, 54, 49, 81, 74, 75, 57, 41, 87, 77,
#                        61, 46, 79, 65, 84, 49, 61, 90, 73, 67, 81, 52, 68, 80, 76, 71, 87, 60, 87, 84, 84, 58, 52, 84,
#                        56, 50, 43, 84, 55, 41, 84, 89, 58, 86, 52, 42, 63, 77, 67, 67, 67, 78, 82, 48, 76, 57, 45, 88,
#                        45, 53, 56, 74]

reception_ratio_gen = [81, 62, 68, 74, 80, 69, 65, 63, 75, 70, 45, 46, 66, 90, 55, 58, 43, 77, 85, 71, 74, 57, 67, 66,
                       83, 46, 89, 79, 83, 74, 60, 48, 89, 70, 57, 54, 60, 88, 45, 79, 48, 70, 87, 57, 53, 76, 69, 87,
                       90, 50, 61, 56, 47, 87, 61, 53, 69, 64, 86, 81, 75, 84, 73, 41, 79, 61, 66, 68, 87, 67, 84, 58,
                       48, 45, 60, 51, 56, 61, 72, 71, 50, 56, 79, 52, 40, 80, 52, 78, 83, 89, 83, 82, 51, 44, 83, 74,
                       87, 74, 60, 46, 45, 65, 71, 62, 41, 78, 73, 76, 74, 68, 59, 43, 61, 68, 89, 77, 62, 86, 43, 67,
                       88, 54, 81, 79, 45, 83, 43, 66, 77, 56, 72, 61, 60, 82, 68, 78, 87, 55, 44, 51, 57, 77, 80, 78,
                       79, 41, 90, 78, 61, 44, 42, 87, 73, 50, 76, 42, 53, 77, 66, 68, 75, 83, 69, 82, 59, 59, 50, 49,
                       74, 52, 58, 90, 86, 70, 46, 53, 72, 49, 89, 85, 47, 82, 69, 55, 73, 74, 67, 64, 70, 84, 48, 68,
                       65, 74, 47, 46, 73, 47, 51, 88]

for i in range(NUMBER_NODES):
    for j in range(len(nodes[i].neighbors)):
        reception_ratio[i][nodes[i].neighbors[j]] = reception_ratio_gen[i]
        reception_ratio[nodes[i].neighbors[j]][i] = reception_ratio_gen[i]

# ------------------------------ Constants ------------------------------------
DATA_RATE = 1000
PACKET_LENGTH = 38
BYTE = 8

# each scan function increases the simulator's current time as much as 
# SCAN_STEP = 0.2 ms
SCAN_STEP = 0.2

# when the node switches between channels, it takes time as much as SWITCH_TIME
SWITCH_TIME = 0.15

# when the node advertises a packet, it takes time as much as Advertise_time
Advertise_time = (PACKET_LENGTH * BYTE) / DATA_RATE

# the array for saving all network events and their time
all_event = []

NODE_TIME = 0  # used for all_event's elements
NODE_EVENT = 1  # Used for all_event's elements

# --------------------------- Defining Events ---------------------------------
GENERATION_EVENT_Adv37 = 1
HEARTBEAT_EVENT_Adv37 = 11
RELAY_EVENT_Adv37 = 2
AD38_EVENT = 3
AD39_EVENT = 4
AD39_EVENT_End = 5
SCAN37_EVENT = 6
SCAN38_EVENT = 7
SCAN39_EVENT = 8
SCAN37_C_EVENT = 60
SCAN38_C_EVENT = 70
SCAN39_C_EVENT = 80
SWITCH_37TO38 = 9
SWITCH_38TO39 = 10
SEND_POLL = 12
FRIEND_RELAY = 13

# -------------------------- Defining Features --------------------------------
SINK_NODE = 0
JUST_RELAY = 1
JUST_GENERATION = 2
RELAY_AND_GENERATION = 3
LOW_POWER = 4
FRIEND_RELAY_NODE = 5  # FRIEND_RELAY_NODE
FRIEND_NODE = 6

# ------------------------- Initialize Logging --------------------------------
# some settings for logging during the simulation

formatter = logging.Formatter('%(message)s')
energy_log = setup_logger('energy_log', 'energy' + '.log')

if collision_log == 1:
    collisions = setup_logger('collisions_log', 'collisions_log.log')
    # collisions.info("Channel    Node   Collisions  Time")

if TOTAL_LOG == 1:
    detail_log = setup_logger('detail_log', 'detail_log.log')
if TOTAL_LOG == 1:
    for init1 in range(NUMBER_NODES):
        logger.append(setup_logger(str(init1), str(init1) + '.log'))
else:
    logger = setup_logger('logger', 'network_detail' + '.log')

# ---------------------- Initialize Events and Time ---------------------------
# the initial times are compared with each other,
# and the event with the minimum time is chosen as the initial event

for init in range(NUMBER_NODES):
    max_seq.append(0)

    # For specified nodes and when the condition is provided
    # Both times (last generation and last scan) are initialized randomly
    if nodes[init].last_generation_time <= nodes[init].last_t_scan and \
            (nodes[init].feature == FRIEND_RELAY_NODE or nodes[init].feature == FRIEND_NODE or nodes[init].feature ==
             JUST_GENERATION or nodes[init].feature == RELAY_AND_GENERATION):
        First_time = nodes[init].last_generation_time  # determining the time of the first event in each node
        First_event = GENERATION_EVENT_Adv37  # determining the first event in each node

    # For all low power nodes
    elif nodes[init].feature == LOW_POWER:
        First_time = nodes[init].last_generation_time
        First_event = GENERATION_EVENT_Adv37

    # Not only for sink and relay node, but also for other types except low power node       
    else:
        First_time = nodes[init].last_t_scan
        First_event = SCAN37_EVENT

    list_node = [First_time, First_event]
    all_event.append(list_node)

# i_node is the node which its event should be done as soon as possible
# The minimum time of events is specified then the event of this time is going to be done.
i_node = all_event.index(min((x for x in all_event), key=lambda k: k[0]))

# the time of i_node event is set as the current time of the simulator
Time = all_event[i_node][NODE_TIME]

# This is used to have mobile nodes and defining their new positions.
rw = random_waypoint(NUMBER_NODES, dimensions=(ENVIRONMENT, ENVIRONMENT),
                     velocity=(0.25, 1.0), wt_max=10.0)

print("initial : ", all_event, sep='\n')

for init_t in range(NUMBER_NODES):
    # First_time of a node, is assigned to the init_time of the node
    nodes[init_t].init_time = all_event[init_t][0]

# # ----------------------------- Relay Node Selection -----------------------------
#
# # Dividing nodes into 16 groups based on their position
# divs = dict()
# steps = 10
# for i in range(NUMBER_NODES):
#     divs[i] = []
#
# for i in range(0, ENVIRONMENT + 1, steps):
#     for k in range(NUMBER_NODES):
#         if i <= nodes[k].Xposition < i + steps:
#             divs[i].append(nodes[k].ID)
#
# divs_list = []
#
# for key in divs:
#     if divs[key] != []:
#         divs_list.append(divs[key])
#
#
# new_list_div = []
# for i in range(len(divs_list)):
#     new_list_div.append([])
#     for j in range(len(divs_list)):
#         new_list_div[i].append([])
#
# for i in range(len(new_list_div)):
#     for m in range(len(divs_list[i])):
#         for k in range(0, ENVIRONMENT + 1, steps):
#             if k <= nodes[m].Yposition < k + steps:
#                 index = k // steps
#                 new_list_div[i][index].append(nodes[m].ID)
#
# count = 0
# for i in range(len(new_list_div)):
#     for j in range(len(new_list_div[i])):
#         count += len(new_list_div[i][j])
# print(count)
# print(new_list_div)

# --------------------------- Plot Network Topology ---------------------------
# print(nx.info(Gar))
fig = figure()  # it is possible to use figsize=(num, num)
color = ['' for i in range(NUMBER_NODES)]
for nd in Gar:
    if nodes[nd].feature == 1:
        color[nd] = 'green'
    elif nodes[nd].feature == 2:
        color[nd] = 'orange'
    else:
        color[nd] = 'red'

    if nodes[nd].feature == 0:
        color[nd] = 'blue'

pos = {}
for i in range(len(Gar.nodes)):
    pos[i] = np.array([nodes[i].Xposition, nodes[i].Yposition])

nx.draw(Gar, pos=pos, with_labels=True, node_color=color)
plt.savefig('topology.png', dpi=200, bbox_inches='tight')
# plt.show()

print(Relay_Node)
