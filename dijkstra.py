import math

graph = {'A': {'D': 15},
         'B': {'I': 9},
         'C': {')': 8},
         'D': {'E': 26, 'K': 62, 'A': 15},
         'E': {'D': 26, 'F': 40},
         'F': {'E': 40, 'G': 21},
         'G': {'F': 21, 'U': 21, 'X': 11},
         'H': {'I': 6},
         'I': {'H': 6, 'B': 9, 'J': 6},
         'J': {'I': 6, 'K': 9, 'O': 13},
         'K': {'J': 9, 'L': 4, 'M': 15, 'D': 62, 'U': 21},
         'L': {'K': 4, 'M': 12},
         'M': {'L': 12, 'K': 15, 'N': 3, 'R': 4},
         'N': {'O': 6, 'M': 3},
         'O': {'J': 13, 'N': 6},
         'Q': {'R': 5},
         'R': {'M': 4, 'Q': 5, 'S': 8, '£': 18},
         'S': {'R': 8},
         'T': {'£': 4},
         'U': {'K': 21, 'G': 21, 'V': 12},
         'V': {'U': 12, '!': 12},
         'W': {'X': 4},
         'X': {'W': 4, 'G': 11, 'Y': 4},
         'Y': {'X': 4, '=': 9, 'Z': 5, '!': 4},
         'Z': {'Y': 5, '=': 8, '-': 6, '+': 3, '%': 5},
         '*': {'(': 5},
         '(': {'*': 5, ')': 6, '$': 5},
         '$': {'(': 5, '£': 5},
         '£': {'$': 5, 'T': 4, 'R': 18, '"': 6},
         ')': {'^': 4, '(': 6, '¬': 5, 'C': 8},
         '^': {'&': 5, ')': 4, '"': 5},
         '"': {'£': 6, '!': 6, '%': 5, '^': 5},
         '¬': {')': 5},
         '&': {'^': 5, '%': 5},
         '%': {'!': 3, 'Z': 5, '&': 5, '"': 5},
         '!': {'V': 12, 'Y': 4, '%': 3, '"': 6},
         '+': {'Z': 3, '-': 5},
         '-': {'+': 5, '=': 5, 'Z': 6},
         '=': {'-': 5, 'Z': 8, 'Y': 9}
         }


class Vertex:

    def __init__(self, name, shortest_path=float("inf")):
        self.name = name
        self.shortest_path = shortest_path
        self.predecessor = None
        self.edges = {}

    def printDetails(self):
        print(f"""
		Name: {self.name}
		Edges: {self.edges}
		Shortest Path: {self.shortest_path}""")

    def findPredecessor(self):
        """
        Find the node that we had to traverse through previously to get to the current node
        """
        # If the node had been traversed through a previous node...
        if self.predecessor:
            # We return the name of that previous node and its previous node
            return self.predecessor.name + self.predecessor.findPredecessor()
        else:
            return ""

    def getShortestPath(self):
        return self.shortest_path


class Graph:

    def __init__(self, graph):

        # Stores the actual graph
        self.graph = []

        # Stores only the node/vertex names
        self.nodes = []

        # Pass the graph list and into this method
        self.createGraph(graph)

        # Ask for the start node and end node
        self.pathBetween = self.getStartAndGoal()

        # Find the shortest path using dijkstra between these nodes
        self.dijkstra(self.pathBetween[0], self.pathBetween[1])

    def getStartAndGoal(self):
        """
        Asks for initial and goal state/node/vertex
        """
        start = input("Enter the start node: ")
        goal = input("Enter the goal node: ")

        return [start, goal]

    def createGraph(self, graph):
        """
        Initialises the graph object by:

        - Creating a Vertex object for each vertex in the graph dictionary argument provided
        - Representing the edges between the vertices by inspecting the graph dictionary argument then modifying each Vertex object
        -
        """
        # For each dictionary item we create a new Vertex object and add it to the nodes list of the Graph object
        for item in graph:
            self.nodes.append(Vertex(item))

        # Iterate through every single dictionary item
        for item in graph:

            # Get the vertex object (stored in the nodes attribute) from its name
            vert = self.vertexByName(item)

            # Iterate through every single edge/connection with the current node
            for edge in graph[item].keys():
                # Add a key value pair to the edges dictionary attribute with the key being the name of the connected node
                # and the value being the weight stored in the graph dictionary
                vert.edges[self.vertexByName(edge)] = graph[item][edge]

            # Add this Vertex object to the graph list attribute
            self.graph.append(vert)

    def vertexByName(self, name):
        """
        Finds the Vertex by simply passing it's name
        """
        for item in self.nodes:
            if name == item.name:
                return item

    def smallestInQueue(self, queue):
        """
        Returns the vertex which is the smallest distance away from its predecessor from a
        list of unexplored vertices (essentially I've tried to implement a priority queue)
        """

        # Assume that the closest any vertex from our current vertex is an infinite distance away
        smallestDist = float("inf")

        # For each vertex in our queue
        for item in queue.keys():

            # If the distance from the current vertex to this vertex is smaller than previous ones then reassign our return variable
            if queue[item] < smallestDist:
                smallest = item
                smallestDist = queue[item]

        return smallest

    def dijkstra(self, start, goal):

        # Keeps track of which nodes are directly connected to the nodes recently discovered
        queue = {}

        # keeps track of which nodes have been found
        found = []

        # We are only given the name of the nodes, we must find the corresponding the Vertex for each node name
        startVertex = self.vertexByName(start)
        goalVertex = self.vertexByName(goal)

        # Set the shortest path of the starting vertex equal to 0 since we are already there
        startVertex.shortest_path = 0

        # Iterate through every node linked to our starting vertex
        for neighbourNode in startVertex.edges.keys():
            # set its shortest path from infinity to the distance between it and the starting vertex
            neighbourNode.shortest_path = startVertex.edges[neighbourNode]

            # set the vertex's predecessor equal to the start vertex so that it knows where it came from
            neighbourNode.predecessor = startVertex

            # add these nodes to the queue since they have been discovered and so we should explore their neighbouring nodes
            queue[neighbourNode] = neighbourNode.shortest_path

        # looping variable, only set false as soon as we encounter our goal vertex or there are no more nodes left to traverse
        notFound = True

        # while the goal vertex hasn't been found or all the nodes haven't been discovered
        while notFound:

            # get the node closest to the start vertex which hasn't had all its neighbours fully discovered yet
            nextNode = self.smallestInQueue(queue)

            # iterate through every neighbouring node of this node
            for neighbourNode in nextNode.edges.keys():

                # if the sum of the distance between 'nextNode' and the start node AND the distance between 'nextNode' and the
                # neighbouring node is LESS THAN the previous distance between the start node and the current neighbouring node
                if nextNode.shortest_path + nextNode.edges[neighbourNode] < neighbourNode.shortest_path:
                    # then we assign a new shortest-peth value as we have a found a path faster to this node
                    neighbourNode.shortest_path = nextNode.shortest_path + nextNode.edges[neighbourNode]

                    # the shortest path is the shortest path from the start node to this parent node plus the path from the
                    # parent node to this child node so we assign the predecessor as the parent node so that we can get the
                    # parent node's predecessor too and find a route
                    neighbourNode.predecessor = nextNode

                # if the neighbouring node hasn't already been discovered
                if neighbourNode not in found:
                    # then we add that node to the queue along with its corresponding shortest path found
                    queue[neighbourNode] = neighbourNode.shortest_path

            # we add the parent node to the found list since we have explored all its linked nodes
            found.append(nextNode)

            # we remove it from the queue since its been found
            del queue[nextNode]

            # if the goal node has already been dsicovered all if all the nodes have already been discovered then we
            # terminate the loop
            if len(queue) == 0 or goalVertex in queue:
                notFound = False

        print(f"The distance between {startVertex.name} and {goalVertex.name} is {goalVertex.getShortestPath()}")
        path = startVertex.name

        preds = goalVertex.findPredecessor()

        for x in range(len(preds) - 2, -1, -1):
            path += " ==> " + preds[x]

        path += " ==> " + goalVertex.name
        print(path)


newGraph = Graph(graph)
