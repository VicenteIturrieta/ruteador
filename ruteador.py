import openrouteservice
from ortools.constraint_solver import pywrapcp, routing_enums_pb2

client = openrouteservice.Client(key="eyJvcmciOiI1YjNjZTM1OTc4NTExMTAwMDFjZjYyNDgiLCJpZCI6IjU5MjExYjU5NjgyNTRhNmM5OTZjY2Q1NzgwMzMwMDc1IiwiaCI6Im11cm11cjY0In0=")

locations = [
    [-72.093775, -36.562653],  # Bodega (Inicio)
    [-72.105695, -36.615563],  # Cliente 1 (Lider Collín)
    [-72.086749, -36.624249],  # Cliente 2 (Acuenta Los Puelches)
    [-72.085021, -36.590386],  # Cliente 3 (Lider express Vicente Mendez)
]

matrix = client.distance_matrix(
    locations=locations,
    profile="driving-hgv", # perfil para vehículos= hgv, car, foot-walking, cycling-regular
    metrics=["duration", "distance"], # duration=tiempo, distance=distancia
    units="m"  # unidades: m=metros, km=kilómetros, mi=millas
)

duration = matrix['durations']  # Segundos
distances = matrix['distances']  # Metros

print("Duraciones (minutos):")
for row in durations:
    print(row)

print("\n\nDistancias (metros):")
for row in distances:
    print(row)

# OR-Tools ruteo
def create_data_model():
    data = {}
    data['time_matrix'] = durations
    data['num_vehicles'] = 1 # número de camiones
    data['depot'] = 0  # punto inicial
    return data

data = create_data_model()

# índices
manager = pywrapcp.RoutingIndexManager(len(data['time_matrix']),
                                       data['num_vehicles'],
                                       data['depot'])

# Modelo de ruteo
routing = pywrapcp.RoutingModel(manager)

# Función de costo (tiempo)
def time_callback(from_index, to_index):
    from_node = manager.IndexToNode(from_index)
    to_node = manager.IndexToNode(to_index)
    return int(data['time_matrix'][from_node][to_node])

transit_callback_index = routing.RegisterTransitCallback(time_callback)
routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

# Parámetros de búsqueda
search_parameters = pywrapcp.DefaultRoutingSearchParameters()
search_parameters.first_solution_strategy = (
    routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

# Resolver
solution = routing.SolveWithParameters(search_parameters)

# solución
if solution:
    index = routing.Start(0)
    plan_output = 'Ruta óptima:\n'
    route_distance = 0
    while not routing.IsEnd(index):
        plan_output += f" {manager.IndexToNode(index)} ->"
        previous_index = index
        index = solution.Value(routing.NextVar(index))
        route_distance += routing.GetArcCostForVehicle(previous_index, index, 0)
    plan_output += f" {manager.IndexToNode(index)}\n"
    plan_output += f"Tiempo total: {route_distance} segundos"
    print(plan_output)
else:
    print("No se encontró solución")