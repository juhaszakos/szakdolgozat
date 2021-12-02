
if __name__ == '__main__':
    # -*- coding: utf-8 -*-

    maps = []  # the matrix storing the maps
    starting_points = []  # the starting point (integer)
    destinations = []  # the array of the destinations
    distances = []  # the distance found by each algortihm for each test
    times = []  # the time it took for each algortihm to find the solution to each problem
    avg_distances = []  # the average distance of the routes each method has found (integer)
    paths = []  #

    maps.append([
        [-1, 2, 3, -1, 2],
        [2, -1, 1, -1, -1],
        [3, 1, -1, 1, -1],
        [-1, -1, 1, -1, 4],
        [2, -1, -1, 4, -1]
    ])
    starting_points.append(3)
    destinations.append([0, 1, 2, 4])

    maps.append([
        [-1, 7, 5, -1, -1, -1, 4, -1, -1, -1, 3, 1, -1],
        [7, -1, 3, -1, -1, 2, -1, -1, -1, -1, -1, -1, -1],
        [5, 3, -1, 3, -1, 3, 2, -1, -1, -1, -1, -1, -1],
        [-1, -1, 3, -1, 2, 4, -1, -1, -1, -1, -1, -1, -1],
        [-1, -1, -1, 2, -1, 2, -1, 6, -1, -1, -1, -1, -1],
        [-1, 2, 3, 4, 2, -1, -1, -1, 7, -1, -1, -1, -1],
        [4, -1, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, 3],
        [-1, -1, -1, -1, 6, -1, -1, -1, -1, 3, -1, -1, -1],
        [-1, -1, -1, -1, -1, 7, -1, -1, -1, 3, -1, -1, -1],
        [-1, -1, -1, -1, -1, -1, -1, 3, 3, -1, -1, -1, -1],
        [3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 4, -1],
        [1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 4, -1, -1],
        [-1, -1, -1, -1, -1, -1, 3, -1, -1, -1, -1, -1, -1]
    ])
    starting_points.append(6)
    destinations.append([0, 1, 2, 3, 4, 5, 7, 8, 9, 10, 11])

    maps.append([
        [-1, 2, -1, -1, 3, -1, -1, -1, -1, 3, -1, -1, -1],
        [2, -1, -1, -1, -1, 3, -1, -1, -1, 1, -1, 3, -1],
        [-1, -1, -1, 4, -1, -1, 3, -1, -1, -1, -1, 2, 3],
        [-1, -1, 4, -1, 4, -1, -1, -1, 7, -1, -1, -1, -1],
        [3, -1, -1, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1],
        [-1, 3, -1, -1, -1, -1, 2, -1, -1, -1, -1, 1, -1],
        [-1, -1, 3, -1, -1, 2, -1, -1, -1, -1, -1, -1, 1],
        [-1, -1, -1, -1, -1, -1, -1, -1, 1, -1, -1, -1, 4],
        [-1, -1, -1, 7, -1, -1, -1, 1, -1, -1, -1, -1, -1],
        [3, 1, -1, -1, -1, -1, -1, -1, -1, -1, 4, -1, -1],
        [-1, -1, -1, -1, -1, -1, -1, -1, -1, 4, -1, -1, -1],
        [-1, 3, 2, -1, -1, 1, -1, -1, -1, -1, -1, -1, -1],
        [-1, -1, 3, -1, -1, -1, 1, 4, -1, -1, -1, -1, -1]
    ])
    starting_points.append(8)
    destinations.append([4, 7, 10, 11])

    maps.append([
        [-1, -1, -1, 7, -1, -1, -1, 9, -1, -1, -1],
        [-1, -1, -1, -1, -1, -1, 11, -1, -1, 3, -1],
        [-1, -1, -1, -1, -1, 5, 7, -1, -1, -1, -1],
        [7, -1, -1, -1, -1, -1, -1, -1, -1, 1, -1],
        [-1, -1, -1, -1, -1, -1, 5, 1, -1, -1, -1],
        [-1, -1, 5, -1, -1, -1, 13, -1, -1, -1, 20],
        [-1, 11, 7, -1, 5, 13, -1, -1, -1, -1, -1],
        [9, -1, -1, -1, 1, -1, -1, -1, -1, -1, 11],
        [-1, -1, -1, -1, -1, -1, -1, -1, -1, 5, -1],
        [-1, 3, -1, 1, -1, -1, -1, -1, 5, -1, -1],
        [-1, -1, -1, -1, -1, 20, -1, 11, -1, -1, -1]
    ])

    starting_points.append(0)
    destinations.append([8, 6, 10])

    maps.append([
        [-1, -1, -1, 1, 1, -1, 5, -1, -1, -1, -1],
        [-1, -1, -1, 5, -1, 7, 4, -1, -1, -1, -1, -1],
        [-1, 5, -1, 4, -1, -1, -1, -1, -1, -1, -1],
        [1, -1, 4, -1, -1, -1, -1, -1, -1, -1, -1],
        [1, 7, -1, -1, -1, -1, -1, -1, -1, 3, -1],
        [-1, 4, -1, -1, -1, -1, -1, 1, 3, -1, -1],
        [5, -1, -1, -1, -1, -1, -1, -1, -1, -1, 1],
        [-1, -1, -1, -1, -1, 1, -1, -1, -1, -1, 5],
        [-1, -1, -1, -1, 1, 3, -1, -1, -1, -1, 4],
        [-1, -1, -1, -1, 3, -1, -1, -1, -1, -1, 1],
        [-1, -1, -1, -1, -1, -1, 1, 5, 4, -1, -1]
    ])

    starting_points.append(4)
    destinations.append([2, 5, 9, 1])

    maps.append([
        [-1, 1, 1, 1, 1, 1],
        [1, -1, 1, 1, 1, 1],
        [1, 1, -1, 1, 1, 1],
        [1, 1, 1, -1, 1, 1],
        [1, 1, 1, 1, -1, 1],
        [1, 1, 1, 1, 1, -1]
    ])
    starting_points.append(0)
    destinations.append([1, 2, 3, 4, 5])

    maps.append([
        [-1, 1, -1, 1, -1, -1, -1, -1, -1, -1, -1, -1],
        [1, -1, 1, 1, -1, 1, -1, -1, -1, -1, -1, -1],
        [-1, 1, -1, -1, -1, 1, -1, -1, -1, -1, -1, -1],
        [1, 1, -1, -1, -1, -1, -1, 1, -1, -1, -1, -1],
        [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
        [-1, 1, 1, -1, -1, -1, -1, 1, 1, -1, -1, -1],
        [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
        [-1, -1, -1, 1, -1, 1, -1, -1, 1, 1, 1, 1],
        [-1, -1, -1, -1, -1, 1, -1, 1, -1, -1, -1, 1],
        [-1, -1, -1, -1, -1, -1, -1, 1, -1, -1, 1, -1],
        [-1, -1, -1, -1, -1, -1, -1, 1, -1, 1, -1, 1],
        [-1, -1, -1, -1, -1, -1, -1, 1, 1, -1, 1, -1]
    ])
    starting_points.append(0)
    destinations.append([1, 2, 3, 5, 7, 8, 9, 10, 11])

    maps.append([
        [-1, 1, -1, 1, 1, -1, -1, -1, -1, -1, -1, -1],
        [1, -1, 1, 1, 1, 1, -1, -1, -1, -1, -1, -1],
        [-1, 1, -1, -1, 1, 1, -1, -1, -1, -1, -1, -1],
        [1, 1, -1, -1, 1, -1, 1, 1, -1, -1, -1, -1],
        [1, 1, 1, 1, -1, 1, 1, 1, 1, -1, -1, -1],
        [-1, 1, 1, -1, 1, -1, -1, 1, 1, -1, -1, -1],
        [-1, -1, -1, 1, 1, -1, -1, 1, -1, 1, 1, -1],
        [-1, -1, -1, 1, 1, 1, 1, -1, 1, 1, 1, 1],
        [-1, -1, -1, -1, 1, 1, -1, 1, -1, -1, 1, 1],
        [-1, -1, -1, -1, -1, -1, 1, 1, -1, -1, 1, -1],
        [-1, -1, -1, -1, -1, -1, 1, 1, 1, 1, -1, 1],
        [-1, -1, -1, -1, -1, -1, -1, 1, 1, -1, 1, -1]
    ])
    starting_points.append(0)
    destinations.append([1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11])

    """
    Uniform Cost Search
    """

    """
    This function loads the undirected graph represented by an NxN (N:= # of nodes) matrix, into
    a dictionary. The keys of the dictionary stand for the nodes in the graph. 
    To each key there is a list appended, that list holds all the nodes which is 
    reachable from that very node.

    @return: a dictionary with the nodes as keys and the reachable nodeas as value
    @param map: a N*N (N:= # of nodes)  matrix representing the graph 
                values: -1: there is no connection between the nodes
                        >0: there is a connection between the nodes with the given cost

     """


    def load_graph(map):

        tmp = []  # this list is used to identify the number of nodes the graph has

        # this loop creates the nodes as keys
        for index in range(len(map)):
            tmp.append(index)

        graph = dict.fromkeys(tmp)  # creating the dictionary for the graph

        # then adding the empty list to the keys
        for index in range(len(map)):
            graph[index] = []

        # this loops trought the matrix
        for row_index in range(len(map)):
            for column_index in range(len(map)):
                # and if cell doesn't contain '-1'
                # the value of the cell is then added to the correct key's list
                if map[row_index][column_index] != -1:
                    graph[row_index].append(column_index)

        return graph


    """
    This function loads the cost from one node to another and represents it with a
    dictionary holding with a key of a tuple of the two nodes, and the value is
    the cost between the nodes .

    @return: a map holing the costs between nodes
            the node pairs are the key
            the cost is the value

    @param in_graph: the graph created by the 'load_graph()' function
    @param map: a N*N (N:= # of nodes)  matrix representing the graph 
                values: -1: there is no connection between the nodes
                        >0: there is a connection between the nodes with the given cost

    """


    def load_cost(in_graph, map):

        tmp = []  # this holds each node pairs temporarly

        # extracting the node pairs from the graph
        for key, value in in_graph.items():
            for item in value:
                tmp.append((key, item))

        cost = dict.fromkeys(tmp)  # creating the dictionary from the node pairs as key

        # adding the costs to the node pairs
        for key in cost:
            cost[key] = map[key[0]][key[1]]

        return cost


    """
    This function is one interpretation of the uniform cost search algorithm.
    This calculates the optimal path costs from start node to all the goal nodes.

    @return: a list in which on all the indexes there is a cost for the same indexed
            goal node

    @param goal: a list containing all the goal nodes
    @param: start an integer representing the start node
    @param graph: the graph created by the 'load_graph()' function
    @param cost: dictionary storing the cost between nodes created by the 'load_cost()' function

    """


    def uniform_cost_search(goal, start, graph, cost):

        count = 0  # this holds the number of all the reached goal nodes
        visited = {}  # this map stores the visited nodes
        queue = []  # this is the priority queue holding pairs of cost and node
        answer = []  # this list holds the costs, this will be returned

        # this loops throughout the goal nodes and creates great values as
        # the base of comparison later in the code
        for i in range(len(goal)):
            answer.append(10 ** 8)

        queue.append([0, start])  # appending the first node with 0 cost, also this
        # is the data structure of its elements

        # looping throught the priority queue to expand the top nodes children
        while (len(queue) > 0):

            queue = sorted(queue)  # sort the queue to make it a priority queue

            # pop the first element
            p = queue[-1]
            del queue[-1]

            p[0] *= -1  # and then retrieve the original cost value

            # check if the node is a goal
            if (p[1] in goal):

                index = goal.index(p[1])

                if (answer[index] == 10 ** 8):
                    count += 1

                if (answer[index] > p[0]):
                    answer[index] = p[0]

                queue = sorted(queue)

                if (count == len(goal)):
                    return answer

            # if the nodes are not visited
            if (p[1] not in visited):
                for i in range(len(graph[p[1]])):
                    # appending the new node with its cost to the priority queue
                    # the cost is multiplyed by -1 so that when sorted the least
                    # costly node will be on top
                    queue.append([(p[0] + cost[(p[1], graph[p[1]][i])]) * -1, graph[p[1]][i]])

            visited[p[1]] = 1  # mark the node as visited

        return answer


    """
    This function is an interpretation of the UCS algorithm. It creates an optimal
    path between the start node and the goal node.

    @return: a list of lists in which the last elements are the path and a value as cost

    @param goal: an integer representing the goal node
    @param start: an integer representing the start node
    @param graph: the graph created by the 'load_graph()' function
    @param cost: dictionary storing the cost between nodes created by the 'load_cost()' function

    """


    def get_path_from_ucs(goal, start, graph, cost):

        queue = []  # create a priority queue
        path = []  # list for all paths, it consist of other lists
        costs = []  # list for the costs
        # for all nodes' costs we append a great number as a base of comparison
        for _ in range(len(graph)):
            costs.append(10 ** 8)

        costs[start] = 0  # the starting nodes cost is 0
        queue.append([0, start])  # insert the starting index, with the first cost

        # the path list init with empty lists
        for _ in range(len(graph)):
            path.append([])

        path[start].append(start)  # the start node's path will first hold itself

        while (len(queue) > 0):

            queue = sorted(queue)  # first we sort the priority queue

            # then pop the first item
            p = queue[-1]
            del queue[-1]

            p[0] *= -1  # we retrieve the original cost

            current = p[1]  # this is teh current node

            # loop throught all the child of the current node
            for child in graph[p[1]]:

                # if the nodes cost is higher then the current node's + the travel cost
                # from current to child
                if costs[child] > costs[current] + cost[(current, child)]:
                    # then create a new node in the queue
                    queue.append([(p[0] + cost[(current, child)]) * -1, child])
                    # update the cost and add the current's path, because it's less costly
                    costs[child] = costs[current] + cost[(current, child)]
                    path[child].append(path[current])
                    path[child].append(child)

        return (path[goal], costs[goal])


    """
    This function creates an optimal route from the starting point to all the goal
    points.

    @return overall_cost: the overall path cost of the optimal path
    @return dropout_order: a list in which the smaller the index the the faster a node will be reached
    @return optimal_path: a list of list in which every list's last element is the next reached node
    @return optimal_costs: a list holding the subroutes' path costs, the costs are ordered by the dropout_order

    @param testIdx: 
    As an input it takes the index number of a map, and a corresponding starting point
    with the goal nodes.
    """


    def find_room_path_via_UCS(testIdx):

        # let's just load everything
        graph = load_graph(maps[testIdx])
        cost = load_cost(graph, maps[testIdx])
        start = starting_points[testIdx]
        goal = destinations[testIdx]
        overall_cost = 0  # this keeps count of all the optimal path costs
        dropout_order = []  # the order in that the goal nodes are reached (first at lower index)

        # loop throught the goal nodes
        while len(goal) > 0:
            # calcualte their cost
            travel_costs = uniform_cost_search(goal, start, graph, cost)
            min_cost = min(travel_costs)
            overall_cost += min_cost
            min_cost_index = travel_costs.index(min_cost)
            dropout_order.append(goal[min_cost_index])
            del goal[min_cost_index]

        optimal_path = []
        optimal_costs = []

        tmp_path, tmp_costs = get_path_from_ucs(dropout_order[0], start, graph, cost)
        optimal_path.append(tmp_path)
        optimal_costs.append(tmp_costs)

        for index in range(1, len(dropout_order)):
            path, costs = get_path_from_ucs(dropout_order[index], dropout_order[index - 1], graph, cost)
            optimal_path.append(path)
            optimal_costs.append(costs)

        overall_cost = 0
        for cost_item in optimal_costs:
            overall_cost += cost_item

        return (overall_cost, dropout_order, optimal_path, optimal_costs)


    """
    THis is a helper function!
    This function takes a list of lists with the usable value as the last element of the lists
    and creates a list with just the usable values (still the list is reversed).

    @return: a list with the values

    @param path_list: the lsit of lists reprezentation from the 'get_path_from_ucs' funtion
    @param buffer: a list
    """


    def get_path_from_list(path_list, buffer):

        if isinstance(path_list, list):
            buffer.append(path_list[-1])
            # print("THIS IS WHAT I APPEND: ",path_list[-1])
            get_path_from_list(path_list[0], buffer)
        # else:
        # buffer.append(path_list)
        # print("[get_path_from_list] this is the buffer before return: ",buffer)
        return buffer


    """
    This function sums up the results.

    @return overall_cost: the overall cost of the path found
    @return dropout_order: the order in which the goal nodes will we reached
    @return ronda_megoldas_path: is the correct path


    @param print: a boolean value. if True then the result will be printed
    @param overall_cost: the overall path cost of the optimal path
    @param dropout_order: a list in which the smaller the index the the faster a node will be reached
    @param optimal_path: a list of list in which every list's last element is the next reached node
    @param optimal_costs: a list holding the subroutes' path costs, the costs are ordered by the dropout_order

    """


    def print_results(_print, overall_cost, dropout_order, optimal_path, optimal_costs):
        if _print == True:
            print("The overall cost of the travel is: ", overall_cost)
            print("The order in which the rooms will be visited: ", dropout_order)
        ronda_megoldas_path = []
        buffer = []
        tmp_buffer = []
        reversed_path = []
        for index in range(len(optimal_costs)):

            buffer = get_path_from_list(optimal_path[index], buffer)
            # print("[print_results] this is buffer after return: ",buffer)

            for item in reversed(buffer):
                # ronda_megoldas_path.append(item)
                reversed_path.append(item)

            tmp_buffer = reversed_path.copy()
            if index != 0:
                del tmp_buffer[0]

            for item in tmp_buffer:
                ronda_megoldas_path.append(item)

            if _print:
                print("The optimal path is: ", reversed_path, " and the cost of this sub-path is: ",
                      optimal_costs[index])
            buffer = []
            reversed_path = []
            tmp_buffer = []
        if _print:
            print("This is the full path: ", ronda_megoldas_path)
            print("")
        return (overall_cost, dropout_order, ronda_megoldas_path)




    print('Paths found by Uniform Cost Search')
    overall_cost, dropout_order, optimal_path, optimal_costs = find_room_path_via_UCS(0)
    overall_cost, _, optimal_path = print_results(True, overall_cost, dropout_order, optimal_path, optimal_costs)


    overall_cost, dropout_order, optimal_path, optimal_costs = find_room_path_via_UCS(1)
    overall_cost, _, optimal_path = print_results(True, overall_cost, dropout_order, optimal_path, optimal_costs)


    overall_cost, dropout_order, optimal_path, optimal_costs = find_room_path_via_UCS(2)
    overall_cost, _, optimal_path = print_results(True, overall_cost, dropout_order, optimal_path, optimal_costs)


    overall_cost, dropout_order, optimal_path, optimal_costs = find_room_path_via_UCS(3)
    overall_cost, _, optimal_path = print_results(True, overall_cost, dropout_order, optimal_path, optimal_costs)


    overall_cost, dropout_order, optimal_path, optimal_costs = find_room_path_via_UCS(4)
    overall_cost, _, optimal_path = print_results(True, overall_cost, dropout_order, optimal_path, optimal_costs)

    overall_cost, dropout_order, optimal_path, optimal_costs = find_room_path_via_UCS(5)
    overall_cost, _, optimal_path = print_results(True, overall_cost, dropout_order, optimal_path, optimal_costs)

    overall_cost, dropout_order, optimal_path, optimal_costs = find_room_path_via_UCS(6)
    overall_cost, _, optimal_path = print_results(True, overall_cost, dropout_order, optimal_path, optimal_costs)

    overall_cost, dropout_order, optimal_path, optimal_costs = find_room_path_via_UCS(7)
    overall_cost, _, optimal_path = print_results(True, overall_cost, dropout_order, optimal_path, optimal_costs)
