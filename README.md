# BMSim
 ## An Event-Driven Simulator for Performance Evaluation of Bluetooth Mesh Networks
BMSim is a simulation framework for BM networks. the main advantages of BMSim simulator are as follows. BMSim is accurate enough that can be trusted for performance evaluation of BM networks (verified by real experiments). Second, low level details of packet transmission and networking procedures (both medium access and network layers) are truly modeled so that various performance metrics can be estimated. Third, the simulator is modular and flexible so that extensions for inclusion of any channel/radio/mobility/interference models are possible and straightforward. As the last and very important specification, it is possible to configure the network and its parameters during simulation so that dynamic networks with run-time configurations mechanisms can be simulated. 

BMSim is an open-source and publicly available event-driven Bluetooth mesh network simulator. Python is used as the programming language for implementation of the simulator. We first introduce the architecture of the simulator, and then discuss its core operation and user interfaces.

\Fig{fig:SimulatorArch} shows the general architecture of BMSim and its inner components. A simulation starts by receiving some high-level network specifications from the user. The Initializer module prepares the first snapshot of the node deployment in the specified simulation area according to user wishes. Then the Updater module runs to prepare required inputs for the simulator engine. The updater module runs every update seconds to support network dynamism as well as run-time configuration changes. The \emph{BM simulation engine} is a discrete event simulator, for which the events and their timings are made according to the BLE and BM standard protocols. This engine continues making and processing events until the requested simulation time ($T_{sim}$) is reached. The Logger module creates the required output files while the simulation engine is running, and finally calculates and reports various performance indicators of the simulated BM network.

### Initializer Module
The initialize module is the entry point user interface for the simulator by which the user initial settings are received and compiled into the simulation framework. It gets the number of nodes in the network (network size, $N$) and the dimensions of the area the nodes should be deployed in. Also, the simulation time ($T_{sim}$) is the time frame that the user expects the network to be simulated. Initializer distributes N instances of the node model all over the simulation area based on the topology parameters given by user; it may be either uniformly random distribution, a regular grid structure, or specifying the x-y coordinations of all nodes. The last case is especially used when several simulation runs are expected to be simulated starting from the same deployment. Anyways, the initializer only determines the initial locations of nodes, which remain fixed all over the simulation time if no node mobility is requested (static networks). In case of having node mobility, the mobility model block of the network updater module determines the new locations of the mobile nodes at each simulation step.

There are a number of parameters related to different models used in the next components of the simulator that are received from user by the initializer module and are directly passed to the corresponding blocks. Parameters of the mobility/channel/radio/interference models, BM stack initial configuration and node types, and the level of expected logs are other inputs of the initializer module. Note that since the simulator is expected to be easily configurable by plugging in/out various models, the exact list of input parameters will differ. For instance, for link extraction, user can simply specify the communication range of nodes or the Packet Reception Ratio (PRR) of links between each pair of nodes to have a high level modeling of channel behavior without simulating details of channel and radio. However, to have more precise simulation, one can use a path-loss model, for instance, for which several parameters such as transmission power of nodes, receiver sensitivity, and channel path-loss exponent need to be set. The same applies to other models such as mobility and interference models.
