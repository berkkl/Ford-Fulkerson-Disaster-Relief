# Dependencies
from collections import defaultdict
import networkx as nx
import matplotlib.pyplot as plt


# Defining Node Class
class Node:
    def __init__(self, name, resources={}):
        self.name = name
        self.resources = resources


# Defining Edge Class
class Edge:
    def __init__(self, u, v, num_of_trucks, truck_capacity, road_multiplier, travel_time, resource_priority):
        self.u = u
        self.v = v
        self.num_of_trucks = num_of_trucks
        self.truck_capacity = truck_capacity
        self.road_multiplier = road_multiplier
        self.travel_time = travel_time
        self.resource_priority = resource_priority


# Defining Truck Class
class Truck:
    def __init__(self, capacity):
        self.capacity = capacity


# Defining Graph Class
class Graph:
    def __init__(self):
        self.graph = defaultdict(dict)

    # Defining add_edge method
    def add_edge(self, edge):
        u, v = edge.u, edge.v
        # Calculate edge capacity considering the factors discussed
        capacity = (edge.num_of_trucks * edge.truck_capacity *
                    edge.road_multiplier * edge.resource_priority)
        if capacity != float("inf"):  # Check if capacity is not infinite before rounding
            capacity = int(capacity)  # Round down the capacity to the nearest integer

        self.graph[u][v] = [capacity, len(self.graph[v])]
        self.graph[v][u] = [0, len(self.graph[u]) - 1]

    # Defining bfs method to find the augmenting path
    def bfs(self, s, t, parent):
        visited = {v: False for v in
                   self.graph}  # initialize a dictionary of visited vertices with False for all vertices

        queue = [s]  # add the source vertex to the queue
        visited[s] = True  # mark the source vertex as visited

        while queue:
            u = queue.pop(0)  # remove the first vertex from the queue

            for v in self.graph[u]:  # iterate over all the neighbors of the current vertex
                if visited[v] == False and self.graph[u][v][
                    0] > 0:  # if the neighbor has not been visited and there is available capacity on the edge
                    queue.append(v)  # add the neighbor to the queue
                    visited[v] = True  # mark the neighbor as visited
                    parent[v] = u  # set the parent of the neighbor to the current vertex

        return visited[t]  # return True if the sink vertex is visited, False otherwise

    # Defining ford_fulkerson method to find the maximum flow
    def ford_fulkerson(self, source, sink):
        parent = {}  # initialize a dictionary to keep track of the parent of each vertex in the augmenting path
        max_flow = 0  # initialize the maximum flow to 0

        while self.bfs(source, sink, parent):  # while there exists an augmenting path from source to sink
            path_flow = float("Inf")  # initialize the path flow to infinity
            s = sink
            while s != source:  # iterate over the vertices in the augmenting path starting from the sink and going backwards towards the source
                path_flow = min(path_flow,
                                self.graph[parent[s]][s][0])  # find the minimum capacity of the edges in the path
                s = parent[s]  # move to the previous vertex in the path

            max_flow += path_flow  # add the path flow to the maximum flow
            v = sink
            while v != source:  # iterate over the vertices in the augmenting path starting from the sink and going backwards towards the source
                u = parent[v]  # get the previous vertex in the path
                forward_edge = self.graph[u][v]  # get the forward edge from u to v
                reverse_edge = self.graph[v][u]  # get the backward edge from v to u
                forward_edge[0] -= path_flow  # decrease the capacity of the forward edge by the path flow
                reverse_edge[0] += path_flow  # increase the capacity of the backward edge by the path flow
                v = parent[v]  # move to the previous vertex in the path

        return max_flow  # return the maximum flow


# example usage
g = Graph()

# Create nodes with resources
s = Node("S")
t = Node("T")
dc1 = Node("DC1", {"water": 1000, "blanket": 50})
dc2 = Node("DC2", {"water": 2000, "blanket": 70})
dc3 = Node("DC3", {"water": 1500, "blanket": 30})
dc4 = Node("DC4", {"water": 800, "blanket": 40})
s1 = Node("S1")
s2 = Node("S2")
s3 = Node("S3")
s4 = Node("S4")
s5 = Node("S5")

# Create trucks
truck = Truck(capacity=1)

# Add edges with capacities and other properties
g.add_edge(Edge("S", "DC1", num_of_trucks=3, truck_capacity=truck.capacity, road_multiplier=1, travel_time=1,
                resource_priority=1))
g.add_edge(Edge("S", "DC2", num_of_trucks=2, truck_capacity=truck.capacity, road_multiplier=1, travel_time=2,
                resource_priority=1))
g.add_edge(Edge("S", "DC3", num_of_trucks=4, truck_capacity=truck.capacity, road_multiplier=1, travel_time=1,
                resource_priority=1))
g.add_edge(Edge("S", "DC4", num_of_trucks=3, truck_capacity=truck.capacity, road_multiplier=1, travel_time=3,
                resource_priority=1))

g.add_edge(Edge("DC1", "S1", num_of_trucks=10, truck_capacity=truck.capacity, road_multiplier=1, travel_time=1,
                resource_priority=1))
g.add_edge(Edge("DC1", "S2", num_of_trucks=2, truck_capacity=truck.capacity, road_multiplier=0.9, travel_time=2,
                resource_priority=1))
g.add_edge(Edge("DC1", "S3", num_of_trucks=5, truck_capacity=truck.capacity, road_multiplier=1, travel_time=2,
                resource_priority=1))
g.add_edge(Edge("DC2", "S2", num_of_trucks=5, truck_capacity=truck.capacity, road_multiplier=0.8, travel_time=1,
                resource_priority=1))
g.add_edge(Edge("DC2", "S3", num_of_trucks=8, truck_capacity=truck.capacity, road_multiplier=1, travel_time=3,
                resource_priority=1))
g.add_edge(Edge("DC2", "S4", num_of_trucks=5, truck_capacity=truck.capacity, road_multiplier=1, travel_time=1,
                resource_priority=1))
g.add_edge(Edge("DC3", "S3", num_of_trucks=4, truck_capacity=truck.capacity, road_multiplier=0.9, travel_time=1,
                resource_priority=1))
g.add_edge(Edge("DC3", "S4", num_of_trucks=6, truck_capacity=truck.capacity, road_multiplier=0.8, travel_time=2,
                resource_priority=1))
g.add_edge(Edge("DC3", "S5", num_of_trucks=5, truck_capacity=truck.capacity, road_multiplier=1, travel_time=3,
                resource_priority=1))
g.add_edge(Edge("DC4", "S4", num_of_trucks=2, truck_capacity=truck.capacity, road_multiplier=1, travel_time=1,
                resource_priority=1))
g.add_edge(Edge("DC4", "S5", num_of_trucks=8, truck_capacity=truck.capacity, road_multiplier=0.7, travel_time=2,
                resource_priority=1))

g.add_edge(Edge("S1", "T", num_of_trucks=float("inf"), truck_capacity=truck.capacity, road_multiplier=1, travel_time=1,
                resource_priority=1))
g.add_edge(Edge("S2", "T", num_of_trucks=float("inf"), truck_capacity=truck.capacity, road_multiplier=1, travel_time=2,
                resource_priority=1))
g.add_edge(Edge("S3", "T", num_of_trucks=float("inf"), truck_capacity=truck.capacity, road_multiplier=1, travel_time=1,
                resource_priority=1))
g.add_edge(Edge("S4", "T", num_of_trucks=float("inf"), truck_capacity=truck.capacity, road_multiplier=1, travel_time=3,
                resource_priority=1))
g.add_edge(Edge("S5", "T", num_of_trucks=float("inf"), truck_capacity=truck.capacity, road_multiplier=1, travel_time=2,
                resource_priority=1))

max_flow_value = g.ford_fulkerson("S", "T")
print("Maximum flow:", max_flow_value)


# Visualize the graph
def draw_graph(graph):
    G = nx.DiGraph()
    pos = {
        "S": (0, 4),
        "DC1": (2, 7),
        "DC2": (2, 4),
        "DC3": (2, 1),
        "DC4": (2, -2),
        "S1": (4, 7),
        "S2": (4, 4),
        "S3": (4, 1),
        "S4": (4, -1),
        "S5": (4, -4),
        "T": (6, 4)
    }

    for u in graph.graph:
        for v in graph.graph[u]:
            capacity = graph.graph[u][v][0]
            G.add_edge(u, v, capacity=capacity)

    edge_labels = {(u, v): f"{d['capacity']}" for u, v, d in G.edges(data=True)}

    # Drawing the graph
    plt.figure(figsize=(12, 8))
    nx.draw(G, pos, with_labels=True, node_size=2500, node_color="#1f78b4", font_size=12, font_weight="bold")
    nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels, font_size=12, font_weight="bold")
    plt.title("Disaster Relief Supply Chain")
    plt.title(f"Maximum flow: {max_flow_value}")
    plt.axis("off")
    plt.show()


draw_graph(g)
