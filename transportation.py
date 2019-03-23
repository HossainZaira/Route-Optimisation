# -*- coding: utf-8 -*-
"""
Multivehicle Routing Problem
Created on Sat Sep 29 18:12:40 2018

@author: zaira
"""

import googlemaps
from googlemaps import Client
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2


manuf_cntr = "459 Jln Ahmad Ibrahim, Singapore 639934" #Beer importers & distributers
distb_cntr1 = "9 Fourth Lok Yang Road, 629706" #Pacific Beverages
distb_cntr2 = "32 Old Toh Tuck Rd, #04-15, Singapore 597658" #Eastern Craft
distb_cntr3 = "40 Jalan Pemimpin, #04-06A Tat Ann Building, 577185"
distb_cntr4 = "100D Pasir Panjang Rd, Singapore 118520" #Radjawali Distribution
distb_cntr5 = "1 Harbourfront Pl, HarbourFront Tower One, Singapore 098633" #JF Hillebrand
distb_cntr6 = "81A Clemenceau Avenue #05-18, 239918" #Mad Tapper
distb_cntr7 = " 7 Kaki Bukit Rd 1, 02-12, Singapore 415937" #Goodman global
distb_cntr8 = "152 Paya Lebar Rd, Singapore 409020" #East of Avalon
distb_cntr9 = "3 Upper Aljunied Link, Singapore 367902" #Wine & Spirits
distb_cntr10 = "190 Tagore Ln, Singapore 787585" #Chuan Seng Huat
distb_cntr11 = "15 Yishun Industrial Street 1, #05-12 Win5, Singapore, 768091" #Rouge Merchants

distb_cntr_list = [distb_cntr1,distb_cntr2,distb_cntr3,distb_cntr4,distb_cntr5,
                   distb_cntr6,distb_cntr7,distb_cntr8,distb_cntr9,distb_cntr10,distb_cntr11]


def runMultiVehicleOptimzation(key,num_vehicles):
    locations = [manuf_cntr,distb_cntr2,distb_cntr3,distb_cntr5,
                 distb_cntr6,distb_cntr8,distb_cntr9,distb_cntr10]
    
    #Create GoogleMap instance to request information from Google Maps API
    gmap = Client(key)
    
    #Fetch inter-location distances from google maps api
    dist_mtrx_resp = gmap.distance_matrix(locations,locations,mode="driving")
    
    #Store the distancevalues in a distance matrix
    x = 0
    y = 0
    dist_mtrx=[[0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0],
               [0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0]        
              ]
    while (x<len(locations)):
        while (y<len(locations)):
            dist_mtrx[x][y]=dist_mtrx_resp.get("rows")[x].get("elements")[y].get("distance").get("value")
            y+=1
        y=0
        x+=1
    
    
    ###################################
    # Distance callback and dimension #
    ####################################
    
    def CreateDistanceCallback(dist_matrix):
      def dist_callback(from_node, to_node):
        return dist_matrix[from_node][to_node]
      return dist_callback
    
    def add_distance_dimension(routing, dist_callback):
      """Add Global Span constraint"""
      distance = "Distance"
      maximum_distance = 90000
      routing.AddDimension(
        dist_callback,
        0, # null slack
        maximum_distance, # maximum distance per vehicle
        True, # start cumul to zero
        distance)
      distance_dimension = routing.GetDimensionOrDie(distance)
      # Try to minimize the max distance among vehicles.
      distance_dimension.SetGlobalSpanCostCoefficient(100)
    
    ####################
    # Get Routes Array #
    ####################
    def get_routes_array(assignment, num_vehicles, routing):
      # Get the routes for an assignent and return as a list of lists.
      routes = []
      for route_nbr in range(num_vehicles):
        node = routing.Start(route_nbr)
        route = []
    
        while not routing.IsEnd(node):
          index = routing.NodeToIndex(node)
          route.append(index)
          node = assignment.Value(routing.NextVar(node))
        routes.append(route)
      return routes
    
    ########
    # Main #
    ########
      
    # Create Routing Model
    routing = pywrapcp.RoutingModel(len(locations), num_vehicles, 0)
    # Define weight of each edge
    dist_callback = CreateDistanceCallback(dist_mtrx)
    routing.SetArcCostEvaluatorOfAllVehicles(dist_callback)
    add_distance_dimension(routing, dist_callback)
    # Setting first solution heuristic (cheapest addition).
    search_parameters = pywrapcp.RoutingModel.DefaultSearchParameters()
    search_parameters.first_solution_strategy = (routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    # Solve the problem.
    assignment = routing.SolveWithParameters(search_parameters)
    routes = get_routes_array(assignment, num_vehicles, routing)
    
    #calculating route lengths
    route_lengths = []
    for route in routes:
        r_len = 0
        prevNode = 0
        for node in route:
            if (node!=0):
                print(prevNode,node)
                r_len += dist_mtrx[prevNode][node]
            prevNode = node
        print(r_len)
        route_lengths.append(r_len)
    print("Routes array:")
    print(routes)
    print("Route lengths:")
    print(route_lengths)
    print("========")
    
    optimizedResp = {"routesArr":routes,"routesLen":route_lengths,"constraints":{"n_veh":num_vehicles}}
    return optimizedResp