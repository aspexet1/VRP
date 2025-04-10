from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def create_data_model():
    """Create data model with risk parameters"""
    data = {}
    data['distance_matrix'] = [
        [0, 10, 15, 20],
        [10, 0, 35, 25],
        [15, 35, 0, 30],
        [20, 25, 30, 0]
    ]
    data['demands'] = [0, 7, 5, 8]
    data['vehicle_capacities'] = [15, 15]
    data['breakdown_prob'] = [      # Probability of breakdown between nodes
        [0.0, 0.1, 0.2, 0.3],
        [0.1, 0.0, 0.15, 0.25],
        [0.2, 0.15, 0.0, 0.2],
        [0.3, 0.25, 0.2, 0.0]
    ]
    data['node_inactive_prob'] = [0.0, 0.05, 0.1, 0.2]  # Probability nodes are inactive
    data['breakdown_cost'] = 100     # Cost per breakdown incident
    data['inactive_penalty'] = 200   # Penalty per inactive node
    data['num_vehicles'] = 2
    data['depot'] = 0
    return data

def add_risk_dimension(routing, manager, data, risk_limit=0.3):
    """Adds combined risk constraints for breakdowns and node inactivation"""
    def risk_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        
        # Arc risk (breakdown probability * cost)
        arc_risk = data['breakdown_prob'][from_node][to_node] * data['breakdown_cost']
        
        # Node risk (inactivation probability * penalty)
        node_risk = data['node_inactive_prob'][to_node] * data['inactive_penalty']
        
        return int((arc_risk + node_risk) * 1000)  # Scale to integer

    risk_index = routing.RegisterTransitCallback(risk_callback)
    routing.AddDimension(
        risk_index,
        0,  # Null slack
        int(risk_limit * 1000),  # Max allowed risk (scaled)
        True,  # Start cumul to zero
        'Risk'
    )
    return routing

def main():
    data = create_data_model()
    manager = pywrapcp.RoutingIndexManager(
        len(data['distance_matrix']),
        data['num_vehicles'],
        data['depot']
    )
    routing = pywrapcp.RoutingModel(manager)

    # Add distance constraint
    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add capacity constraint
    def demand_callback(from_index):
        from_node = manager.IndexToNode(from_index)
        return data['demands'][from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # Null capacity slack
        data['vehicle_capacities'],
        True,  # Start cumulative to zero
        'Capacity'
    )

    # Add risk constraints
    routing = add_risk_dimension(routing, manager, data, risk_limit=0.3)

    # Allow node skipping with inactivation penalties
    penalty = 0
    for node in range(1, len(data['distance_matrix'])):
        penalty = int(data['node_inactive_prob'][node] * data['inactive_penalty'] * 1000)
        routing.AddDisjunction([manager.NodeToIndex(node)], penalty)

    # Solve with guided local search
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    search_parameters.time_limit.seconds = 30

    solution = routing.SolveWithParameters(search_parameters)

    # Print solution
    if solution:
        print_solution(data, manager, routing, solution)

def print_solution(data, manager, routing, solution):
    """Prints routes with risk and capacity information"""
    total_distance = 0
    total_risk = 0
    total_load = 0

    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        route_load = 0
        route_risk = 0
        plan_output = f'Route for vehicle {vehicle_id}:\n'
        
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            next_index = solution.Value(routing.NextVar(index))
            route_load += data['demands'][node_index]
            
            # Get risk accumulation
            risk_dimension = routing.GetDimensionOrDie('Risk')
            route_risk += solution.Value(risk_dimension.CumulVar(index))
            
            plan_output += f' {node_index} ->'
            index = next_index

        plan_output += f' {manager.IndexToNode(index)}\n'
        plan_output += f'Distance: {solution.Value(routing.CostVar())}m\n'
        plan_output += f'Load: {route_load}\n'
        plan_output += f'Risk: {route_risk/1000:.2f}\n'
        print(plan_output)
        
        total_distance += solution.Value(routing.CostVar())
        total_risk += route_risk

    print(f'Total distance: {total_distance}m')
    print(f'Total risk: {total_risk/1000:.2f}')

if __name__ == '__main__':
    main()
