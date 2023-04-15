# Ford-Fulkerson Disaster Relief Optimization
This is a Python implementation of the Ford-Fulkerson algorithm for optimizing disaster relief supply chain allocation. The code is written for a Graduation Project at Ankara University and can be used as a starting point for similar optimization problems.

### Algorithm
The Ford-Fulkerson algorithm is a classic algorithm for finding the maximum flow in a flow network. In this problem, we represent the disaster relief supply chain as a flow network and use the algorithm to optimize the allocation of resources to the affected areas.

### Usage
The code defines three classes: Node, Edge, and Truck. It also defines a Graph class, which is used to represent the flow network.

To use the code, create instances of the Node, Edge, and Truck classes to represent the nodes, edges, and trucks in the supply chain, respectively. Then, create an instance of the Graph class and add the edges to the graph using the add_edge method.

Finally, call the ford_fulkerson method on the Graph object, passing in the source and sink nodes as arguments. The method returns the maximum flow in the network.

The code includes an example usage, which can be modified as needed to suit the specific problem being solved.

### ILP Optimization
It is worth noting that this algorithm can be extended with Integer Linear Programming (ILP) optimization to allocate resources in trucks. This can be done by introducing binary variables to represent which resources are assigned to which trucks, and adding constraints to ensure that each truck does not exceed its capacity and that each affected area receives the required resources.
