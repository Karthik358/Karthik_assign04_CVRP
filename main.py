"""Capacited Vehicles Routing Problem (CVRP)."""

from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

import sys
import json
with open('assignment_cvrp.json', 'r') as file:
    inputdata = json.load(file)

# Extract matrices
weight_matrix = inputdata["weight_matrix"]
volume_matrix = inputdata["volume_matrix"]
location_matrix = inputdata["location_matrix"]

# Initialize dictionaries to store cumulative values for each location
cumulative_weights = {}
cumulative_volumes = {}

# Iterate through the location matrix
for i, loc in enumerate(location_matrix):
    if loc not in cumulative_weights:
        cumulative_weights[loc] = 0
        cumulative_volumes[loc] = 0
    
    cumulative_weights[loc] += weight_matrix[i]
    cumulative_volumes[loc] += volume_matrix[i]

# Print the results
# for loc in cumulative_weights:
#     print(f"{loc}: Weight = {cumulative_weights[loc]}, Volume = {cumulative_volumes[loc]}")
# Initialize lists to store cumulative weights and volumes in order of loc0 to loc8
ordered_weights = [0] * 8  # 8 locations from loc0 to loc7
ordered_volumes = [0] * 8

# Define the order of locations
locations_order = ["loc0", "loc1", "loc2", "loc3", "loc4", "loc5", "loc6", "loc7"]

# Populate the ordered lists
for i, loc in enumerate(locations_order):
    if loc in cumulative_weights:
        ordered_weights[i] = cumulative_weights[loc]
        ordered_volumes[i] = cumulative_volumes[loc]

def create_data_model():
    """Stores the data for the problem."""
    data = {}
    data["distance_matrix"] = inputdata["distance"]
    data["demands"] = ordered_weights #inputdata["weight_matrix"]
    data["demand_vol"] = ordered_volumes #inputdata["volume_matrix"]
    data["vehicle_capacities"] = inputdata["max_weight"]
    data["vehicle_volumes"] = inputdata["max_volume"]
    data["fixedCostPerVehicle"] = inputdata["fixedCostPerVehicle"]
    data["perKmCostPerVehicle"] = inputdata["perKmCostPerVehicle"]
    data["num_vehicles"] = 6
    data["depot"] = 0
    return data

route_dict_list = []
def print_solution(data, manager, routing, solution):
    """Prints solution on console."""
    print(f"Objective: {solution.ObjectiveValue()}")
    total_distance = 0
    total_load = 0
    total_vol = 0
    route_id = 0
    for vehicle_id in range(data["num_vehicles"]):
        route_dict={}
        index = routing.Start(vehicle_id)
        plan_output = f"Route for vehicle {vehicle_id}:\n"
        route_distance = 0
        route_load = 0
        route_vol = 0
        
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            route_vol += data["demand_vol"][node_index]
            route_load += data["demands"][node_index]
            plan_output += f" {node_index} Weight({route_load}) Volume({route_vol}) -> "
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id
            )
        plan_output += f" {manager.IndexToNode(index)} Weight({route_load}) Volume({route_vol})\n"
        plan_output += f"Cost of the route: {route_distance}\n"
        plan_output += f"Weight of the route: {route_load}\n"
        plan_output += f"Volume of the route: {route_vol}\n"
        print(plan_output)
        total_distance += route_distance
        total_load += route_load
        total_vol += route_vol

        route_dict.update({'route_id': route_id,
        'route_weight' : route_load,
        'route_volume' : route_vol,
        'vehicle_id' : vehicle_id,
        'veh_max_weight' : data["vehicle_capacities"][vehicle_id],
        'veh_max_volume' : data["vehicle_volumes"][vehicle_id],
        'route_cost' : route_distance})
        route_dict_list.append(route_dict)
        route_id+=1

    print(f"Total cost of all routes: {total_distance}")
    print(f"Total weight of all routes: {total_load}")
    print(f"Total volume of all routes: {total_vol}")

    
def main():
    """Solve the CVRP problem."""
    # Instantiate the data problem.
    data = create_data_model()

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(
        len(data["distance_matrix"]), data["num_vehicles"], data["depot"]
    )

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback.
    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data["distance_matrix"][from_node][to_node]

    # transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # # Define cost of each arc.
    # routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    def vehicle_cost_callback(vehicle_id,from_index,to_index):
        """Returns the cost between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data["perKmCostPerVehicle"][vehicle_id]*data["distance_matrix"][from_node][to_node]
    
    for vehicle_id in range(data["num_vehicles"]):
        vehicle_cost_callback_index = routing.RegisterTransitCallback(lambda from_index,to_index,vehicle_id=vehicle_id:vehicle_cost_callback(vehicle_id,from_index,to_index))
        routing.SetArcCostEvaluatorOfVehicle(vehicle_cost_callback_index,vehicle_id)
        veh_fixed_cost = data["fixedCostPerVehicle"][vehicle_id]
        routing.SetFixedCostOfVehicle(veh_fixed_cost,vehicle_id)
    # Add Capacity constraint.
    def demand_callback(from_index):
        """Returns the demand of the node."""
        # Convert from routing variable Index to demands NodeIndex.
        from_node = manager.IndexToNode(from_index)
        return data["demands"][from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        data["vehicle_capacities"],  # vehicle maximum capacities
        True,  # start cumul to zero
        "Capacity",
    )

    # Add Volume constraint.
    def demand_callback(from_index):
        """Returns the demand of the node."""
        # Convert from routing variable Index to demands NodeIndex.
        from_node = manager.IndexToNode(from_index)
        return data["demand_vol"][from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        data["vehicle_volumes"],  # vehicle volume capacities
        True,  # start cumul to zero
        "Volume",
    )

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    )
    search_parameters.time_limit.FromSeconds(1)

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Print solution on console.
    if solution:
        print_solution(data, manager, routing, solution)


if __name__ == "__main__":
    main()
    out_file = open("output.json", "w")
    json.dump(route_dict_list, out_file, indent = 6)
    out_file.close()