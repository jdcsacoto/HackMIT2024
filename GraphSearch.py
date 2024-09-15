import heapq
class Node():
    def __init__(self, name, floor, entrance = False):
        self.name = name
        self.floor = floor
        self.edges = []
        self.entrance = entrance

    def add_edge(self, other, weight, message, angle, up = 0):
        self.edges.append((other, weight, message, angle, up))

    def get_edges(self):
        for edge in self.edges:
            yield edge

    def __lt__(self, other):
        return self.name < other.name

class Graph():
    def __init__(self, nodes):
        self.nodes = nodes

    def dijkstra(self, start_node, end_node):
        # Initialize distances with infinity, except the start node
        distances = {node: float('inf') for node in self.nodes}
        distances[start_node] = 0

        # Priority queue for processing nodes
        priority_queue = [(start_node, 0)]

        # Dictionary to track the shortest path
        shortest_path = {}

        # finds shortest path to all nodes until the end is the shortest
        while priority_queue:
            # takes the closest node
            current_node, current_distance = heapq.heappop(priority_queue)
            if current_node == end_node:
                return distances, shortest_path

            # throws away distances longer than shortest
            if current_distance > distances[current_node]:
                continue

            # checks all neighboring nodes
            for neighbor, weight, message, angle, up in current_node.get_edges():
                distance = current_distance + weight

                # records shortest distance
                if distance < distances[neighbor]:
                    distances[neighbor] = distance
                    shortest_path[neighbor] = (current_node, message, angle, up)
                    heapq.heappush(priority_queue, (neighbor, distance))

        return distances, shortest_path

    def get_shortest_path(self, start_node, end_node):
        _, shortest_path = self.dijkstra(start_node, end_node)
        path = []
        angles = []
        floors = []

        nodes = []

        current_node = shortest_path.get(end_node)

        # iterates backwards to find shortest path
        while current_node:
            path.append(current_node[1])
            angles.append(current_node[2])
            floors.append((current_node[0].floor, current_node[0].entrance, current_node[3]))
            current_node = shortest_path.get(current_node[0])

        # processes message
        truths = [f"go to {path[-1]}"]
        for i in range(len(path)-2, 0, -1):
            if floors[i][1]:
                truths.append("enter " + path[i])
            elif floors[i][2] > 0:
                truths.append(f"go up to {path[i]}")
            elif floors[i][2] < 0:
                truths.append(f"go down to {path[i]}")
            elif (angles[i]-angles[i+1]+180) % 360 - 180 > 30:
                truths.append("turn left to " + path[i])
            elif (angles[i]-angles[i+1]+180) % 360 - 180 < -30:
                truths.append("turn right to " + path[i])

        truths.append("enter " + path[0])
        return truths

generate_map = True
if generate_map:
    # Create nodes
    h26_100_1 = Node("h26.100_1", 1)
    h26_100_2 = Node("h26.100_2", 1)
    r26_100e1 = Node("26.100e1", 1, True)
    r26_100e2 = Node("26.100e2", 1, True)
    r26_100_in = Node("26.100", 1)
    r26_100_out = Node("26.100", 1)
    rbanana_loungee1 = Node("Banana Loungee1", 1, True)
    rbanana_loungee2 = Node('Banana Loungee2', 1, True)
    rbanana_lounge_in = Node("Banana Lounge", 1)
    rbanana_lounge_out = Node("Banana Lounge", 1)

    e3 = Node("26.100 Entrance", 1, True)

    h26_hub = Node("h26hub", 1)

    s16_1 = Node("Building 16 stairs 1-1.5 low", 1)
    s16_15 = Node("Building 16 stairs 1-1.5 high", 1)

    h8 = Node("h8", 1)

    h6 = Node("h6", 1)
    r6_120 = r6_120_in = r6_120_out = Node("6.120", 1)


    # Add edges
    h26_100_1.add_edge(r26_100e1, 2, "26.100 northern entrance", 0)
    h26_100_1.add_edge(rbanana_loungee1, 2, "The Banana Lounge northern entrance", 180)
    h26_100_1.add_edge(h26_100_2, 10, "a hallway in building 26, floor 1", -90)
    h26_100_2.add_edge(r26_100e2, 2, "26.100 southern entrance", 0)
    h26_100_2.add_edge(rbanana_loungee2, 2, "The Banana Lounge southern entrance", 180)
    h26_100_2.add_edge(h26_100_1, 10, "a hallway in building 26, floor 1", 90)
    h26_100_2.add_edge(h26_hub, 10, "a hallway in building 26, floor 1", -90)
    r26_100e1.add_edge(r26_100_in, 0, "26.100", 0)
    r26_100e1.add_edge(h26_100_1, 2, "a hallway in building 26, floor 1", 180)
    r26_100e2.add_edge(r26_100_in, 0, "26.100", 0)
    r26_100e2.add_edge(h26_100_2, 2, "a hallway in building 26, floor 1", 180)
    r26_100_out.add_edge(h26_100_1, 0, "a hallway in building 26, floor 1", 180)
    r26_100_out.add_edge(h26_100_2, 0, "a hallway in building 26, floor 1", 180)
    rbanana_loungee1.add_edge(h26_100_1, 2, "a hallway in building 26, floor 1", 0)
    rbanana_loungee1.add_edge(rbanana_lounge_in, 0, "The Banana Lounge", 180)
    rbanana_loungee2.add_edge(h26_100_2, 2, "a hallway in building 26, floor 1", 0)
    rbanana_loungee2.add_edge(rbanana_lounge_in, 0, "The Banana Lounge", 180)
    rbanana_lounge_out.add_edge(h26_100_1, 0, "a hallway in building 26, floor 1", 0)
    rbanana_lounge_out.add_edge(h26_100_2, 0, "a hallway in building 26, floor 1", 0)
    e3.add_edge(h26_100_1, 2, "a hallway in building 26, floor 1", -90)
    h26_100_1.add_edge(e3, 2, "a hallway in building 26, floor 1", -90)

    h26_100_2.add_edge(h26_hub, 60, "a hallway in building 26, floor 1", -90)
    h26_hub.add_edge(h26_100_2, 60, "a hallway in building 26, floor 1", 90)
    h26_hub.add_edge(s16_1, 30, "a staircase in building 26, floor 1", -90)
    s16_1.add_edge(h26_hub, 30, "a staircase in building 26, floor 1", 90)
    s16_1.add_edge(s16_15, 7, "a hallway in building 12, floor 1", 180, 1)
    s16_15.add_edge(s16_1, 7, "a hallway in building 12, floor 1", 0, -1)
    s16_15.add_edge(h8, 30, "a hallway in building 8, floor 1", -90)
    h8.add_edge(s16_15, 30, "a hallway in building 8, floor 1", 90)

    h8.add_edge(h6, 100, "a hallway in building 6, floor 1", -90)
    h6.add_edge(h8, 100, "a hallway in building 8, floor 1", 90)
    h6.add_edge(r6_120, 5, "6.120", 180)

    r6_120.add_edge(h6, 5, "6.120", 0)

    # Create graph
    graph = Graph([h26_100_1, h26_100_2,
                   r26_100e1, r26_100e2, r26_100_in, r26_100_out,
                   rbanana_loungee1, rbanana_loungee2, rbanana_lounge_in, rbanana_lounge_out,
                   e3, h26_hub, s16_1, s16_15, h8, h6, r6_120])

def run_navigation(input_1, input_2):

    # turns front end inputs into back end variables
    if input_1[:8] == "Entrance":
        start_str = "e" + input_1[-1]
    else:
        input_1 = input_1.lower()
        start_str = "r"
        for char in input_1:
            if char == ".":
                start_str += "_"
            else:
                start_str += char
        start_str += "_out"

    input_2 = input_2.lower()
    end_str = "r"
    for char in input_2:
        if char == ".":
            end_str += "_"
        else:
            end_str += char
    end_str += "_in"

    # parses variables
    start = eval(start_str)
    end = eval(end_str)

    # Finds shortest path
    path = graph.get_shortest_path(start, end)
    global the_path
    the_path = path
    return path

print(run_navigation("6.120","26.100"))
