import osmnx as ox
import networkx as nx
import queue
import math
import priority_dict

map_graph = ox.graph_from_place('Berkeley, California', network_type='drive')
origin = ox.get_nearest_node(map_graph, (37.8743, -122.277))    #Starting location or Starting point(GOAL)
destination = list(map_graph.nodes())[-1]       #Ending location or Ending point (DESTINATION)

shortest_path = nx.shortest_path(map_graph, origin, destination, weight='length') 
fig, ax = ox.plot_graph_route(map_graph, shortest_path)     #Original Path


#Implementation of Dijkshtra's Algorithm !!!!!!!!
def dijkshtra(origin_key, goal_key, graph):
    
    # The priority queue of open vertices we've reached.
    # Keys are the vertex keys, vals are the distances.
    open_queue = priority_dict.priority_dict({})
    
    #Dictionary to take note of the visited nodes.
    closed_dict = {}
    
    #Dictionary to keep track of predecessors
    predecessors = {}
    
    # Add the origin to the open queue.
    open_queue[origin_key] = 0.0

    # Iterate through the open queue, until we find the goal.
    # Each time, perform a Dijkstra's update on the queue.
    # TODO: Implement the Dijstra update loop.
    goal_found = False
    while (open_queue): #Loop to go through each node.

        vertexU, vertexUcost = open_queue.pop_smallest()
        
        #If vertexU is the goal_key,then break.
        if vertexU == goal_key:
            goal_found = True
            break
        
        #If vertexU is not the goal_key...
        #For loop to go through all the child nodes of the vertexU
        for edge in graph.out_edges([vertexU], data=True):
            
            #If the vertex is already visited then ignore it....
            #Here edge[1] is the vertexV
            if edge[1] in closed_dict.keys(): 
                continue
            
            vertexUVCost = edge[2]['length'] #The cost from vertexU to vertexV is vertexUVcost and is given by edge[2]['length']
            #If vertexV is there in the open_queqe then initialize the cost of vertexV from open_queue
            if edge[1] in open_queue:
                vertexVcost=open_queue[edge[1]]
                if vertexUcost + vertexUVCost < vertexVcost:   #Optimal cost
                    
                    open_queue[edge[1]] = vertexUcost + vertexUVCost
                    predecessors[edge[1]] = vertexU
                    
            else:
                open_queue[edge[1]] = vertexUcost + vertexUVCost
                predecessors[edge[1]] = vertexU
                
        closed_dict[vertexU]=1 #Visited vertexU is then added to the closed dictionary since all its child nodes has been visited..
            
    # If we get through entire priority queue without finding the goal,then there's some error or Goal is not there in the map.
    if not goal_found:
        raise ValueError("Goal not found in search.")
    
    # Construct the path from the predecessors dictionary.
    return get_path(origin_key, goal_key, predecessors)                


#Function to get the path from the algo.
def get_path(origin_key, goal_key, predecessors):
    currentNode = goal_key
    path=[]
    
    while (currentNode != origin_key):
        path.insert(0, currentNode)
        currentNode = predecessors[currentNode]
        
    path.insert(0,origin_key)
    return path

path_main = dijkshtra(origin, destination, map_graph)      #Getting the path from Dijkshtra's Algorithm  
fig, ax = ox.plot_graph_route(map_graph, path_main)        #Printing the graph for Dijkshtra's Algorithm  


# Computes the Euclidean distance between two vertices.
# Assume that the earth is a sphere with radius 6371 km.
def distance_heuristic(state_key, goal_key, node_data):
    n1 = node_data[state_key]
    n2 = node_data[goal_key]

    # Get the longitude and latitude for each vertex.
    long1 = n1['x']*math.pi/180.0
    lat1 = n1['y']*math.pi/180.0
    long2 = n2['x']*math.pi/180.0
    lat2 = n2['y']*math.pi/180.0
    
    # Use a spherical approximation of the earth for
    # estimating the distance between two points.
    r = 6371000
    x1 = r*math.cos(lat1)*math.cos(long1)
    y1 = r*math.cos(lat1)*math.sin(long1)
    z1 = r*math.sin(lat1)

    x2 = r*math.cos(lat2)*math.cos(long2)
    y2 = r*math.cos(lat2)*math.sin(long2)
    z2 = r*math.sin(lat2)

    d = ((x2-x1)**2 + (y2-y1)**2 + (z2-z1)**2)**0.5
    
    return d

#Implementation of A* Algorithm !!!!!!!!
def a_star_search(origin_key, goal_key, graph):

    # The priority queue of open vertices we've reached.
    # Keys are the vertex keys, vals are the accumulated distances plus the heuristic estimates of the distance to go.
    open_queue = priority_dict.priority_dict({})
    
    #Dictionary to take note of the visited nodes.
    closed_dict = {}
    
    #Dictionary to keep track of predecessors
    predecessors = {}
    
    # The dictionary that stores the best cost to reach each vertex found so far.
    costs = {}
    
    # Get the spatial data for each vertex as a dictionary.
    node_data = graph.nodes(True)
    
    # Add the origin to the open queue and the costs dictionary.
    costs[origin_key] = 0.0
    open_queue[origin_key] = distance_heuristic(origin_key, goal_key, node_data)
    # Iterate through the open queue, until we find the goal.
    # Each time, perform a A* update on the queue.
    # TODO: Implement the A* update loop.
    goal_found = False
    while(open_queue):

        vertexU, vertexU_heuristic = open_queue.pop_smallest()

        

        #If vertexU is the goal_key,then break.
        if vertexU == goal_key:
            goal_found = True
            break
        
        #If vertexU is not the goal_key...
        #For loop to go through all the child nodes of the vertexU
        for edge in graph.out_edges([vertexU], data=True):
            
            #If the vertex is already visited then ignore it....
            #Here edge[1] is the vertexV
            if edge[1] in closed_dict.keys(): 
                continue

            vertexUcost = costs[vertexU]
            vertexUVCost = edge[2]['length'] #The cost from vertexU to vertexV is vertexUVcost and is given by edge[2]['length']
          
            #If vertexV is there in the open_queqe then set the cost of vertexV from open_queue
            if edge[1] in open_queue:
                
                vertexVcost=costs[edge[1]]
                if vertexUcost + vertexUVCost < vertexVcost:   #Optimal cost
                    costs[edge[1]]= vertexUcost + vertexUVCost
                    open_queue[edge[1]] = vertexUcost + vertexUVCost + distance_heuristic(edge[1],goal_key,node_data)
                    predecessors[edge[1]] = vertexU
                    
            else:
                open_queue[edge[1]] = vertexUcost + vertexUVCost + distance_heuristic(edge[1],goal_key,node_data)
                predecessors[edge[1]] = vertexU
                costs[edge[1]]= vertexUcost + vertexUVCost
                
        closed_dict[vertexU]=1 #Visited vertexU is then added to the closed dictionary since all its child nodes has been visited..
            
    # If we get through entire priority queue without finding the goal,then there's some error or Goal is not there in the map.
    if not goal_found:
        raise ValueError("Goal not found in search.")
    
    # Construct the path from the predecessors dictionary.
    return get_path(origin_key, goal_key, predecessors)                



path_a_star = a_star_search(origin, destination, map_graph)     #Getting the path from A* Algorithm  
fig, ax = ox.plot_graph_route(map_graph, path_a_star)           #Printing the graph for A* Algorithm  

##################################################################################################################################################################