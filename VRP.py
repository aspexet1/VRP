from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def create_data_model():
    """Create data model with risk parameters."""
    data = {}
    # Distance matrix (cost of traveling between nodes)
    data['distance_matrix'] = [
        [0, 10, 15, 20],  # From depot (node 0) to nodes 1-3
        [10, 0, 35, 25],  # From node 1 to other nodes
        [15, 35, 0, 30],  # From node 2 to other nodes
        [20, 25, 30, 0]   # From node 3 to other nodes
    ]
    # Demands at each node (depot has demand = 0)
    data['demands'] = [0, 7, 5, 8]
    # Vehicle capacities
    data['vehicle_capacities'] = [15, 15]
    # Breakdown probabilities for arcs (probability of vehicle breakdown between nodes)
    data['breakdown_prob'] = [
        [0.0, 0.1, 0.2, 0.3],   # Probabilities from depot (node 0)
        [0.1, 0.0, 0.15, 0.25], # Probabilities from node 1
        [0.2, 0.15, 0.0, 0.2],  # Probabilities from node 2
        [0.3, 0.25, 0.2, 0.0]   # Probabilities from node 3
    ]
    # Node inactivation probabilities (probability that a node becomes inactive)
    data['node_inactive_prob'] = [0.0, 0.05, 0.1, 0.2]
    # Cost per breakdown incident
    data['breakdown_cost'] = 100
    # Penalty cost for skipping an inactive node
    data['inactive_penalty'] = 200
    # Number of vehicles available
    data['num_vehicles'] = 2
    # Depot index (starting and ending point for all vehicles)
    data['depot'] = 0
    return data

def add_risk_dimension(routing, manager, data, risk_limit=0.3):
    """Adds a risk dimension to account for breakdown and inactivation risks."""
    
    def risk_callback(from_index, to_index):
        """Calculate risk for traveling between two nodes."""
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        
        # Arc risk: Breakdown probability * breakdown cost
        arc_risk = data['breakdown_prob'][from_node][to_node] * data['breakdown_cost']
        
        # Node risk: Inactivation probability * penalty cost
        node_risk = data['node_inactive_prob'][to_node] * data['inactive_penalty']
        
        return int((arc_risk + node_risk) * 1000)  # Scale risk to integer

    # Register the risk callback with OR-Tools
    risk_index = routing.RegisterTransitCallback(risk_callback)
    
    # Add a dimension for cumulative risk along each route
    routing.AddDimension(
        risk_index,
        slack_max=0,                      # No slack allowed for risk accumulation
        capacity=int(risk_limit * 1000),   # Maximum allowed cumulative risk (scaled)
        fix_start_cumul_to_zero=True,     # Start cumulative risk at zero for all routes
        name='Risk'
    )
    
def main():
    """Main function to solve the risk-aware CVRP."""
    
    # Create the problem data model.
    data = create_data_model()
    
    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(
        len(data['distance_matrix']),      # Number of locations (nodes)
        data['num_vehicles'],              # Number of vehicles available
        data['depot']                      # Depot index (starting and ending point)
    )
    
    # Create the routing model.
    routing = pywrapcp.RoutingModel(manager)

    # Define the cost function: Distance matrix as the arc cost.
    def distance_callback(from_index, to_index):
        """Returns the distance between two nodes."""
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    
    # Set the arc cost evaluator for all vehicles.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add capacity constraint.
    def demand_callback(from_index):
        """Returns the demand at a given node."""
        from_node = manager.IndexToNode(from_index)
        return data['demands'][from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
    
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        slack_max=0,                       # No slack allowed for capacity overflows
        vehicle_capacities=data['vehicle_capacities'],   # Vehicle capacity limits
        fix_start_cumul_to_zero=True,
        name='Capacity'
    )

    # Add risk constraints using the custom risk dimension.
    add_risk_dimension(routing, manager, data)

    # Allow skipping nodes with penalties based on inactivation probabilities.
    for node in range(1, len(data['distance_matrix'])):   # Skip depot (node index=0)
        penalty = int(data['node_inactive_prob'][node] * data['inactive_penalty'] * 1000)
        routing.AddDisjunction([manager.NodeToIndex(node)], penalty)

    # Set search parameters.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)   # Initial solution strategy
    
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)   # Optimization strategy
    
    search_parameters.time_limit.seconds = 30   # Time limit for solving

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    if solution:
        print_solution(data, manager, routing, solution)

def print_solution(data, manager, routing, solution):
    """Prints the solution: routes and associated costs."""
    
    total_distance = 0
    total_risk = 0

    for vehicle_id in range(data['num_vehicles']):
        
        index = routing.Start(vehicle_id)   # Start of the route for this vehicle
        
        route_distance = 0
        route_risk = []
        
        plan_output = f'Route for vehicle {vehicle_id}:\n'
        
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            next_index = solution.Value(routing.NextVar(index))
            
            route_distance += data['distance_matrix'][manager.IndexToNode(index)][manager.IndexToNode(next_index)]
            
            plan_output += f' {node_index} ->'
            
            index = next_index
        
        
if __name__
