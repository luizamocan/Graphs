import random
import copy



class DirectedGraph:
    def __init__(self):
        """
                Initializes an empty directed graph.
        """
        self.adjacency_list={} #key:vertex , value: set of outbound neighbors
        self.inbound_list={} #key:vartex, value: set of inbound neighbors
        self.edge_costs={} #key:(source,target), value:cost
        self.num_vertices=0

    def read_from_file(self, filename):
        """
               Reads a graph from a file.
               Args:
                   filename (str): The name of the file to read from.
               Raises:
                   ValueError: If the file format is incorrect.
        """
        with open(filename, 'r') as f:
            first_line = f.readline().strip()
            n,m=map(int,first_line.split())

            self.num_vertices=n

            for _ in range(m):
                line=f.readline().strip()
                source_vertex, destination_vertex, edge_cost=map(int,line.split())
                self.add_edge(source_vertex, destination_vertex, edge_cost)

            for v in range(n):
                self.add_vertex(v)

    def write_to_file(self, filename):
        """
          Writes the graph to a file.
          Args:
              filename (str): The name of the file to write to.
          """
        with open(filename, 'w') as f:
            f.write(f"{len(self.adjacency_list)} {len(self.edge_costs)}\n")
            for (x,y), cost in self.edge_costs.items():
                f.write(f"{x} {y} {cost}\n")

    def add_vertex(self, vertex):
        """
              Adds a vertex to the graph if it does not already exist.
              Args:
                  vertex (int): The vertex to add.
        """
        if vertex not in self.adjacency_list:
            self.adjacency_list[vertex]=set()
            self.inbound_list[vertex]=set()

    def remove_vertex(self, vertex):
        """
             Removes a vertex and all associated edges from the graph.
             Args:
                 vertex (int): The vertex to remove.
             Raises:
                 ValueError: If the vertex does not exist.
         """
        if vertex not in self.adjacency_list:
            raise ValueError('Vertex does not exist')


        outbound_neighbors = list(self.adjacency_list[vertex])
        for neighbor in outbound_neighbors:
            del self.edge_costs[(vertex, neighbor)]
            self.inbound_list[neighbor].remove(vertex)

        inbound_neighbors = list(self.inbound_list[vertex])
        for neighbor in inbound_neighbors:
            del self.edge_costs[(neighbor, vertex)]
            self.adjacency_list[neighbor].remove(vertex)

        del self.adjacency_list[vertex]
        del self.inbound_list[vertex]

    def add_edge(self, source, destination, cost):
        """
             Adds a directed edge from source to destination with the given cost.
            Args:
                source (int): The starting vertex.
                destination (int): The ending vertex.
                cost (int): The cost of the edge.
            Raises:
                ValueError: If the edge already exists.
        """
        if (source, destination) not in self.edge_costs:
            self.add_vertex(source)
            self.add_vertex(destination)
            self.adjacency_list[source].add(destination)
            self.inbound_list[destination].add(source)
            self.edge_costs[(source, destination)] = cost
        else :
            raise ValueError('Edge already exists')


    def remove_edge(self, source, destination):
        """
               Removes an edge from the graph.
               Args:
                   source (int): The starting vertex.
                   destination (int): The ending vertex.
               Raises:
                   ValueError: If the edge does not exist.
        """
        if (source, destination) in self.edge_costs:
            self.adjacency_list[source].remove(destination)
            self.inbound_list[destination].remove(source)
            del self.edge_costs[(source, destination)]
        else :
            raise ValueError("There is no edge between source and destination")


    def get_vertices(self):
        """
        Returns a list of all vertices in the graph.
        Returns:
            list: The list of vertices.
        """
        return list(self.adjacency_list.keys())

    def get_edges(self):
        """
        Prints all edges in the graph along with their costs.
        Format:
            source -> destination : cost

        """
        for (source, destination), cost in self.edge_costs.items():
            print(f"{source} -> {destination} : {cost}")

    def get_number_of_vertices(self):
        """
        Returns the number of vertices in the graph.
        Returns:
            int: The number of vertices.
        """
        return len(self.adjacency_list)

    def edge_exists(self, source, destination):
        """
        Checks if an edge from source to destination exists.
        Args:
            source (int): The starting vertex.
            destination (int): The ending vertex.
        Returns:
            bool: True if the edge exists, False otherwise.
        """
        return (source, destination) in self.edge_costs

    def get_in_degree(self, vertex):
        """
            Returns the in-degree of a given vertex.
            Args:
                vertex (int): The vertex whose in-degree is to be found.
            Returns:
                int: The number of inbound edges for the vertex.
            Raises:
                ValueError: If the vertex does not exist.
        """

        if vertex  not in self.adjacency_list:
           raise ValueError('Vertex does not exist')
        return len(self.inbound_list.get(vertex, set()))

    def get_out_degree(self, vertex):
        """
            Returns the out-degree of a given vertex.
            Args:
                vertex (int): The vertex whose out-degree is to be found.
            Returns:
                int: The number of outbound edges for the vertex.
            Raises:
                ValueError: If the vertex does not exist.
        """
        if vertex not in self.adjacency_list:
            raise ValueError('Vertex does not exist')
        return len(self.adjacency_list.get(vertex, set()))

    def get_outbound_edges(self, vertex):
        """
        Returns a list of all outbound edges from a given vertex.
        Args:
            vertex (int): The vertex whose outbound edges are to be retrieved.
        Returns:
            list: A list of outbound neighbors.
        Raises:
            ValueError: If the vertex does not exist.
        """
        if vertex not in self.adjacency_list:
            raise ValueError('Vertex does not exist')
        return list(self.adjacency_list.get(vertex, set()))

    def get_inbound_edges(self, vertex):
        """
        Returns a list of all inbound edges to a given vertex.
        Args:
            vertex (int): The vertex whose inbound edges are to be retrieved.
        Returns:
            list: A list of inbound neighbors.
        Raises:
            ValueError: If the vertex does not exist.
        """
        if vertex not in self.adjacency_list:
            raise ValueError('Vertex does not exist')
        return list(self.inbound_list.get(vertex, set()))

    def get_edge_cost(self, source, destination):
        """
        Returns the cost of an edge between two vertices.
        Args:
            source (int): The starting vertex.
            destination (int): The ending vertex.
        Returns:
            int or None: The cost of the edge if it exists, otherwise None.
        """
        # If the edge doesn't exist, raise an exception to make it clear
        if (source, destination) not in self.edge_costs:
            raise ValueError(f"No edge exists between {source} and {destination}")
        return self.edge_costs[(source, destination)]

    def set_edge_cost(self, source, destination, cost):
        """
        Sets the cost of an edge if it exists.
        Args:
            source (int): The starting vertex.
            destination (int): The ending vertex.
            cost (int): The new cost of the edge.
        Raises:
            ValueError: If the edge does not exist.
        """
        if (source, destination) not in self.edge_costs:
            raise ValueError("There is no edge between source and destination")
        self.edge_costs[(source, destination)] = cost

    def create_new_graph(self):
        """
        Creates a deep copy of the graph.
        Returns:
            DirectedGraph: A new graph instance with the same structure.
        """
        new_graph = DirectedGraph()
        new_graph.adjacency_list = copy.deepcopy(self.adjacency_list)
        new_graph.inbound_list = copy.deepcopy(self.inbound_list)
        new_graph.edge_costs = copy.deepcopy(self.edge_costs)
        return new_graph


    def get_outbound_edges_iterator(self, vertex):
        """
        Returns an iterator over outbound edges from a given vertex.
        Args:
            vertex (int): The vertex.
        Raises:
            ValueError: If the vertex does not exist.
        """
        if vertex not in self.adjacency_list:
            raise ValueError("Vertex does not exist")
        for neighbor in self.adjacency_list[vertex]:
            yield vertex, neighbor

    def get_inbound_edges_iterator(self, vertex):
        """
        Returns an iterator over inbound edges to a given vertex.
        Args:
            vertex (int): The vertex.
        Raises:
            ValueError: If the vertex does not exist.
        """
        if vertex not in self.inbound_list:
            raise ValueError("Vertex does not exist")
        for neighbor in self.inbound_list[vertex]:
            yield neighbor, vertex

    @staticmethod
    def generate_random_graph(nr_vertices, nr_edges):
        """
        Generates a random directed graph.
        Args:
            nr_vertices (int): Number of vertices.
            nr_edges (int): Number of edges.
        Returns:
            DirectedGraph: A randomly generated graph.
        """
        graph = DirectedGraph()
        all_edges = [(x, y) for x in range(nr_vertices) for y in range(nr_vertices)]
        random.shuffle(all_edges)
        selected_edges = all_edges[:nr_edges]
        for x, y in selected_edges:
            c = random.randint(1, 100)
            graph.add_edge(x, y, c)

        return graph

    def bfs(self, start):
        """
        Performs Breadth-First Search (BFS) from the given start vertex.

        Args:
            start (int): The starting vertex for the BFS.

        Returns:
            tuple: A pair of dictionaries:
                - dist: Maps each reachable vertex to its distance (number of steps) from the start.
                - parent: Maps each vertex to its parent in the BFS traversal (used to reconstruct paths).

        Raises:
            ValueError: If the start vertex does not exist in the graph.
        """
        if start not in self.adjacency_list:
            raise ValueError("Start vertex does not exist\n")

        q = [start]
        dist = {start: 0}
        parent = {start: None}

        while len(q) > 0:
            x = q.pop(0)
            for y in self.get_outbound_edges(x):
                if y not in dist:
                    dist[y] = dist[x] + 1
                    parent[y] = x
                    q.append(y)

        return dist, parent

    @staticmethod
    def retrieve_path(parent, destination):
        """
        Reconstructs the path from the BFS parent map from the start to the destination.

        Args:
            parent (dict): A dictionary mapping each vertex to its parent in the BFS traversal.
            destination (int): The target vertex for which to reconstruct the path.

        Returns:
            list or None: The path from the start to the destination as a list of vertices,
                          or None if the destination is unreachable.
        """
        if destination not in parent:
            return None

        path = []
        x = destination
        while x is not None:
            path.append(x)
            x = parent[x]
        path.reverse()
        return path

    def shortest_path_bfs(self, start, end):
        """
        Computes the shortest path (minimum number of steps) between two vertices using BFS.

        Args:
            start (int): The starting vertex.
            end (int): The target vertex.

        Returns:
            tuple: A pair (length, path), where:
                - length (int): The number of edges in the shortest path.
                - path (list): The list of vertices along the path from start to end.

        Raises:
            ValueError: If start or end vertices do not exist, or if no path exists between them.
        """
        if start not in self.adjacency_list or end not in self.adjacency_list:
            raise ValueError("Start or end vertex does not exist \n")

        dist, parent = self.bfs(start)
        path = self.retrieve_path(parent, end)

        if path is None:
            raise ValueError("There is no path between start and end vertex \n")

        return len(path) - 1, path

    def lowest_cost_walk_backwards_dijkstra(self, start, end):
        """
        Finds the lowest cost walk between start and end vertices using a "backwards" Dijkstra algorithm.
        The search is done in reverse, starting from the end vertex and traversing inbound edges.

        Args:
            start (int): The starting vertex.
            end (int): The target vertex.

        Returns:
            tuple: A pair (cost, path), where:
                - cost (int): The cost of the lowest cost walk.
                - path (list): The list of vertices along the path from start to end.

        Raises:
            ValueError: If start or end vertex does not exist, or no path exists.
        """
        if start not in self.adjacency_list or end not in self.adjacency_list:
            raise ValueError("Start or end vertex does not exist")

        # Initialize the data structures
        dist = {v: float('inf') for v in self.adjacency_list}
        dist[end] = 0
        parent = {end: None}
        unvisited = set(self.adjacency_list.keys())

        while unvisited:
            # Find the vertex with the smallest distance in the unvisited set
            current = min(unvisited, key=lambda vertex: dist[vertex])
            if current == start:
                break

            unvisited.remove(current)

            for neighbor in self.get_inbound_edges(current):
                try:
                    cost = self.get_edge_cost(neighbor, current)
                except ValueError:
                    continue
                if dist[neighbor] > dist[current] + cost:
                    dist[neighbor] = dist[current] + cost
                    parent[neighbor] = current

        # If there's no path from start to end
        if dist[start] == float('inf'):
            raise ValueError("No path exists between the given vertices")

        # Reconstruct the path from start to end
        path = []
        current = start
        while current is not None:
            path.append(current)
            current = parent.get(current)

        return dist[start], path

    def dfs_visit(self, vertex, visited, rec_stack):
        visited[vertex] = True
        rec_stack[vertex] = True

        for neighbor in self.adjacency_list.get(vertex, []):
            if not visited[neighbor]:
                if self.dfs_visit(neighbor, visited, rec_stack):
                    return True
            elif rec_stack[neighbor]:
                return True

        rec_stack[vertex] = False
        return False

    def is_dag(self):

        visited = {vertex: False for vertex in self.adjacency_list}
        rec_stack = {vertex: False for vertex in self.adjacency_list}

        for vertex in self.adjacency_list:
            if not visited[vertex]:
                if self.dfs_visit(vertex, visited, rec_stack):
                    return False
        return True


    def tarjan_dfs(self,vertex,visited,rec_stack,topological_order):
        visited[vertex] = True
        rec_stack[vertex] = True

        for neighbor in self.adjacency_list.get(vertex,[]):
            if not visited[neighbor]:
                self.tarjan_dfs(neighbor,visited,rec_stack,topological_order)
            elif rec_stack[neighbor] :
                raise ValueError("The graph is not a DAG, it contains cycle")

        rec_stack[vertex] = False
        topological_order.append(vertex)

    def topological_sort(self):
        visited={vertex: False for vertex in self.adjacency_list}
        rec_stack={vertex: False for vertex in self.adjacency_list}
        topological_order=[]

        for vertex in self.adjacency_list:
            if not visited[vertex]:
                self.tarjan_dfs(vertex,visited,rec_stack,topological_order)

        topological_order.reverse()
        return topological_order

    def highest_cost_path(self,start,end):
        topological_order = self.topological_sort()
        dist={vertex: float('-inf') for vertex in self.adjacency_list}
        prev={vertex: None for vertex in self.adjacency_list}
        dist[start] = 0

        for vertex in topological_order:
            if dist[vertex] != float('-inf'):
                for neighbor in self.adjacency_list.get(vertex,[]):
                    edge_cost=self.get_edge_cost(vertex,neighbor)
                    if dist[vertex] +edge_cost > dist[neighbor]:
                        dist[neighbor] = dist[vertex] + edge_cost
                        prev[neighbor] = vertex

        if dist[end] == float('-inf'):
            raise ValueError("No path exists between the given vertices")

        #backtrack to find the path
        path=[]
        current = end
        while current is not None:
            path.append(current)
            current = prev[current]
        path.reverse()

        return dist[end], path

    def bellman_ford(self, start, end):
        """
        Finds the minimum cost path from start to end using the Bellman-Ford algorithm.
        Returns:
            (cost, path): cost is the total minimum cost, path is a list of vertices from start to end.
        Raises:
            ValueError: If a negative cycle is detected.
        """
        distance = {v: float('inf') for v in self.get_vertices()}
        predecessor = {v: None for v in self.get_vertices()}
        distance[start] = 0

        for _ in range(self.get_number_of_vertices() - 1):
            updated = False
            for (u, v), cost in self.edge_costs.items():
                if distance[u] + cost < distance[v]:
                    distance[v] = distance[u] + cost
                    predecessor[v] = u
                    updated = True
            if not updated:
                break  # Early stop if no changes

        # Check for negative cost cycles
        for (u, v), cost in self.edge_costs.items():
            if distance[u] + cost < distance[v]:
                raise ValueError("Graph contains a negative weight cycle")


        if distance[end] == float('inf'):
            return float('inf'), []

        path = []
        current = end
        while current is not None:
            path.append(current)
            current = predecessor[current]
        path.reverse()

        return distance[end], path
