
from DirectedGraph import DirectedGraph
class UI:
    def __init__(self):
        self.graph = DirectedGraph()
        self.file_in = "ex.txt"
        self.file_out = "ex-changed.txt"
        DirectedGraph.read_from_file(self.graph, self.file_in)

    @staticmethod
    def print_menu():
        print("1. Get the number of vertices")
        print("2. Iterate the set of vertices")
        print("3. Print the edges")
        print("4. Given two vertices, find if there is an edge from the first one to the second one")
        print("5. Get the in degree and out degree of a vertex")
        print("6. Iterate the set of outbound edges of a vertex")
        print("7. Parse the set of inbound edges of a vertex")
        print("8. Modify the information attached to an edge")
        print("9. Add a new vertex")
        print("10. Remove a vertex")
        print("11. Add a new edge")
        print("12. Remove a edge")
        print("13. Make a copy of the graph")
        print("14. Given  two vertices, find a lowest length path between them")
        print("15. Given two vertices, find the lowest cost path between them (backwards Dijkstra)")
        print("16. Check if the graph is a DAG")
        print("17. Topological sort of a DAG")
        print("18. Highest cost path in a DAG from a source to destination")
        print("19. Find a minimum cost path between 2 vertices (negative cost cycles may exist in the graph)")
        print("0. Exit the program")

    def menu(self):

        while True:
            self.print_menu()
            option = input("Enter your option: ")
            if option == "1":
                print("The number of vertices is: ")
                print(DirectedGraph.get_number_of_vertices(self.graph))

            elif option == "2":
                print("The set of vertices is: ")
                print(DirectedGraph.get_vertices(self.graph))
            elif option == "3":
                print("The edges are: ")
                DirectedGraph.get_edges(self.graph)

            elif option=="4":
                source_vertex=int(input("Enter the source vertex: "))
                destination_vertex=int(input("Enter the destination vertex: "))
                if DirectedGraph.edge_exists(self.graph,source_vertex, destination_vertex):
                    print("Exists an edge between the source vertex and destination vertex ")
                else:
                    print("There isn't an edge between the source vertex and destination vertex")

            elif option == "5":
                try:
                    vertex=int(input("Enter the  vertex: "))
                    print("The in degree of the vertex is: ")
                    print(DirectedGraph.get_in_degree(self.graph,vertex))
                    print("The out degree of the vertex is: ")
                    print(DirectedGraph.get_out_degree(self.graph,vertex))
                except ValueError as ve:
                    print(ve)


            elif option == "6":
                try:
                    vertex = int(input("Enter the vertex: "))
                    print("Outbound edges:")
                    for edge in DirectedGraph.get_outbound_edges_iterator(self.graph,vertex):
                        print(edge)
                except ValueError as ve:
                    print(ve)

            elif option == "7":
                try:
                    vertex = int(input("Enter the vertex: "))
                    print("Inbound edges:")
                    for edge in DirectedGraph.get_inbound_edges_iterator(self.graph, vertex):
                        print(edge)
                except ValueError as ve:
                    print(ve)


            elif option == "8":
                try:
                    source_vertex=int(input("Enter the source vertex: "))
                    destination_vertex=int(input("Enter the destination vertex: "))
                    new_cost=int(input("Enter the new cost: "))
                    DirectedGraph.set_edge_cost(self.graph,source_vertex,destination_vertex,new_cost)
                    print("The new cost was changed successfully")
                    self.graph.write_to_file(self.file_out)
                except ValueError as ve:
                    print(ve)

            elif option == "9":
                DirectedGraph.add_vertex(self.graph, DirectedGraph.get_number_of_vertices(self.graph))
                print("The new vertex was added to the graph")
                self.graph.write_to_file(self.file_out )


            elif option == "10":
                try:
                    remove_vertex=int(input("Enter the vertex to be removed: "))
                    DirectedGraph.remove_vertex(self.graph,remove_vertex)
                    print("The vertex was removed successfully")
                    self.graph.write_to_file(self.file_out)
                except ValueError as ve:
                    print(ve)

            elif option == "11":
                try:
                    source_vertex=int(input("Enter the source vertex: "))
                    destination_vertex=int(input("Enter the destination vertex: "))
                    edge_cost=int(input("Enter the cost: "))
                    DirectedGraph.add_edge(self.graph,source_vertex,destination_vertex,edge_cost)
                    print("The new edge was added to the graph")
                    self.graph.write_to_file(self.file_out)
                except ValueError as ve:
                    print(ve)

            elif option == "12":
                try:
                    source_vertex=int(input("Enter the source vertex: "))
                    destination_vertex=int(input("Enter the destination vertex: "))
                    DirectedGraph.remove_edge(self.graph,source_vertex,destination_vertex)
                    print("The vertex was removed successfully")
                    self.graph.write_to_file(self.file_out)
                except ValueError as ve:
                    print(ve)

            elif option == "13":
                new_graph=DirectedGraph.create_new_graph(self.graph)
                print("The new graph was created successfully")
                file_copy="copy_graph.txt"
                new_graph.write_to_file(file_copy)

            elif option == "14":
                try:
                    source_vertex=int(input("Enter the source vertex: "))
                    destination_vertex=int(input("Enter the destination vertex: "))
                    shortest_distance, path=DirectedGraph.shortest_path_bfs(self.graph,source_vertex,destination_vertex)
                    print("Shortest distance: ", shortest_distance)
                    print("Path: ", path )
                    print()
                except ValueError as ve:
                    print(ve)
            elif option == "15":
                try:
                    source=int(input("Enter the source vertex: "))
                    destination=int(input("Enter the destination vertex: "))
                    shortest_distance,path=DirectedGraph.lowest_cost_walk_backwards_dijkstra(self.graph,source,destination)
                    print("Shortest distance: ", shortest_distance)
                    print("Path: ", path )
                except ValueError as ve:
                    print(ve)
            elif option == "16":
                try:
                    if DirectedGraph.is_dag(self.graph):
                        print("The graph is a DAG")
                    else:
                        print("The graph is not a DAG")
                except ValueError as ve:
                    print(ve)

            elif option == "17":
                try:
                    topological_order=DirectedGraph.topological_sort(self.graph)
                    print("Topological order: ", topological_order)
                except ValueError as ve:
                    print(ve)

            elif option == "18":
                try:
                    source_vertex=int(input("Enter the source vertex: "))
                    destination_vertex=int(input("Enter the destination vertex: "))
                    cost,path=DirectedGraph.highest_cost_path(self.graph,source_vertex,destination_vertex)
                    print("Highest cost path: ", cost)
                    print("Path: ", path)
                except ValueError as ve:
                    print(ve)

            elif option == "19":
                try:
                    source_vertex=int(input("Enter the source vertex: "))
                    destination_vertex=int(input("Enter the destination vertex: "))
                    cost,path=DirectedGraph.bellman_ford(self.graph,source_vertex,destination_vertex)
                    print("Minimum cost path: ", cost)
                    print("Path: ", path)
                except ValueError as ve:
                    print(ve)


            elif option == "0":
                break

            else:
                print("Invalid option. Please try again")

def generate_random_graphs():
    random_graph1=DirectedGraph.generate_random_graph(7,20)
    random_graph2=DirectedGraph.generate_random_graph(6,40)
    DirectedGraph.write_to_file(random_graph1,"random_graph1.txt")
    DirectedGraph.write_to_file(random_graph2,"random_graph2.txt")




if __name__ == "__main__":
    UI().menu()
    generate_random_graphs()