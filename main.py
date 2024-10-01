import time
from search.algorithms import State
from search.map import Map
import getopt
import sys
import heapq 

class Dijkstra(object):
    def __init__(self, start_state, end_state, gridded_map):
        '''
        inputs: start = start_state, end = goal_state, map = gridded_map
        return: none
        function: initalize the varibales 
        '''
        self.start = start_state
        self.end = end_state
        self.map = gridded_map
    
    def calc(self):
        '''
        inputs: None its all done in the class __init__ object where start = start_state, end = goal_state, map = gridded_map
        return: Optimal cost and number of expanded nodes
        function: Find the optimal cost and return it while using Dijkstra algorithm 
        '''
        open = []
        heapq.heappush(open, (0, self.start))  
        closed = {}  # only used for graph visualization of each algo
        cost_dict = {self.start: 0}  
        expansions = 0  
          
        while len(open) > 0:
            cost_n, n = heapq.heappop(open) 
            expansions += 1 
            
            if n == self.end:
                self.map.plot_map(closed, self.start, self.end, "d.png")
                return (cost_dict[self.end], expansions) 

            for n_prime in self.map.successors(n):
                cost = cost_n + abs(self.map.cost(n_prime.get_x() - n.get_x(), n_prime.get_y() - n.get_y()))

                if n_prime not in cost_dict or cost < cost_dict[n_prime]:
                    cost_dict[n_prime] = cost
                    closed[n_prime] = n_prime  
                    heapq.heappush(open, (cost, n_prime))  

        self.map.plot_map(closed, self.start, self.end, "d.png")
        return (-1, expansions) 
    

    
class A_star(object):
    
    def __init__(self, start_state, end_state, gridded_map):
        '''
        inputs: start = start_state, end = goal_state, map = gridded_map
        return: none
        function: initalize the varibales 
        '''
        self.start = start_state
        self.end = end_state
        self.map = gridded_map
    
    def calc(self):
        '''
        inputs: None its all done in the class __init__ object where start = start_state, end = goal_state, map = gridded_map
        return: Optimal cost and number of expanded nodes
        function: Find the optimal cost and return it while using a* algorithm 
        '''
        open = []
        heapq.heappush(open, (0, self.start))  # (f-value, node)
        cost_dict = {self.start: 0} 
        closed = {} #only used for graph visualization of each algo
        expansions = 0

        while open:

            cost_n, n = heapq.heappop(open)  # pop node with lowest f-value

            expansions +=1
            if n == self.end:
                self.map.plot_map(closed, self.start, self.end, "d.png")
                return (cost_dict[self.end], expansions) 
            n_x, n_y = n.get_x(), n.get_y()
            cost = cost_dict[n]

            for n_prime in self.map.successors(n):
                g_n_prime = cost + self.map.cost(n_prime.get_x() - n_x, n_prime.get_y() - n_y)
                delta_x = abs(n_prime.get_x() - self.end.get_x())
                delta_y = abs(n_prime.get_y() - self.end.get_y())
                h_n_prime = 1.5 * min(delta_x, delta_y) + abs(delta_x - delta_y) # using the h(n) formula

                f_n_prime = g_n_prime + h_n_prime

                if n_prime not in cost_dict or g_n_prime < cost_dict[n_prime]:
                    cost_dict[n_prime] = g_n_prime  
                    closed[n_prime] = n_prime
                    heapq.heappush(open, (f_n_prime, n_prime))  

        return (-1, expansions) 


def main():

    """
    Function for testing your A* and Dijkstra's implementation. 
    Run it with a -help option to see the options available. 
    """
    optlist, _ = getopt.getopt(sys.argv[1:], 'h:m:r:', ['testinstances', 'plots', 'help'])

    plots = False
    for o, a in optlist:
        if o in ("-help"):
            print("Examples of Usage:")
            print("Solve set of test instances and generate plots: main.py --plots")
            exit()
        elif o in ("--plots"):
            plots = True

    test_instances = "test-instances/testinstances.txt"
    
    # Dijkstra's algorithm and A* should receive the following map object as input
    gridded_map = Map("dao-map/brc000d.map")
    
    nodes_expanded_dijkstra = []  
    nodes_expanded_astar = []

    time_dijkstra = []  
    time_astar = []

    start_states = []
    goal_states = []
    solution_costs = []
       
    file = open(test_instances, "r")
    for instance_string in file:
        list_instance = instance_string.split(",")
        start_states.append(State(int(list_instance[0]), int(list_instance[1])))
        goal_states.append(State(int(list_instance[2]), int(list_instance[3])))
        
        solution_costs.append(float(list_instance[4]))
    file.close()
        
    for i in range(0, len(start_states)):    
        start = start_states[i]
        goal = goal_states[i]

        gridded_map = Map("dao-map/brc000d.map")
    
        time_start = time.time()
        # print(start_states)
        # print(start_states[i]) # [x,y]
        dijkstra = Dijkstra(start_states[i], goal_states[i],gridded_map) # added line
        cost, expanded_diskstra = dijkstra.calc()  # replace None, None with the call to your Dijkstra's implementation
        time_end = time.time()
        # print(time_end - time_start)

        time_dijkstra.append(time_end - time_start)

        if cost != solution_costs[i]:
            print("There is a mismatch in the solution cost found by Dijkstra and what was expected for the problem:")
            print("Start state: ", start)
            print("Goal state: ", goal)
            print("Solution cost encountered: ", cost)
            print("Solution cost expected: ", solution_costs[i])
            print()    

        start = start_states[i]
        goal = goal_states[i]
    
        time_start = time.time()
        a_star = A_star(start_states[i], goal_states[i], gridded_map)
        cost, expanded_astar = a_star.calc() # replace None, None with the call to your A* implementation
        time_end = time.time()

        nodes_expanded_astar.append(expanded_astar)
        nodes_expanded_dijkstra.append(expanded_diskstra)
        time_astar.append(time_end - time_start)

        if cost != solution_costs[i]:
            print("There is a mismatch in the solution cost found by A* and what was expected for the problem:")
            print("Start state: ", start)
            print("Goal state: ", goal)
            print("Solution cost encountered: ", cost)
            print("Solution cost expected: ", solution_costs[i])
            print()
    #plots = True

    if plots:
        from search.plot_results import PlotResults
        plotter = PlotResults()
        plotter.plot_results(nodes_expanded_astar, nodes_expanded_dijkstra, "Nodes Expanded (A*)", "Nodes Expanded (Dijkstra)", "nodes_expanded")
        plotter.plot_results(time_astar, time_dijkstra, "Running Time (A*)", "Running Time (Dijkstra)", "running_time")

if __name__ == "__main__":
    main()