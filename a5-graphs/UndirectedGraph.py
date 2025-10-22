class Graph:
    def __init__(self, n):
        self.n = n
        self.adj = [[] for _ in range(n)]

    def add_edge(self, u, v, cost):
        self.adj[u].append((v, cost))

    def min_cost_path(self, start, end):
        self.min_cost = float('inf')
        self.best_path = []
        self.visited = [False] * self.n

        def backtrack(u, cost, path):
            if cost > self.min_cost:
                return
            if u == end:
                if cost < self.min_cost:
                    self.min_cost = cost
                    self.best_path = path[:]
                return
            self.visited[u] = True
            for (nbr, c) in self.adj[u]:
                if not self.visited[nbr]:
                    path.append(nbr)
                    backtrack(nbr, cost + c, path)
                    path.pop()
            self.visited[u] = False

        backtrack(start, 0, [start])
        if self.min_cost == float('inf'):
            return None, None  # no path found
        return self.min_cost, self.best_path


if __name__ == "__main__":
    g = Graph(5)
    g.add_edge(0, 1, 2)
    g.add_edge(1, 2, -5)
    g.add_edge(2, 3, 1)
    g.add_edge(3, 4, 2)
    g.add_edge(1, 4, 10)
    g.add_edge(0, 2, 4)

    start = int(input("Enter start vertex \n"))
    end = int(input("Enter end vertex \n"))
    cost, path = g.min_cost_path(start, end)
    if path is None:
        print("No path found")
    else:
        print("Min cost:", cost)
        print("Path:", path)
