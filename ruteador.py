import openrouteservice
from openrouteservice import convert
from ortools.constraint_solver import pywrapcp, routing_enums_pb2
import folium

client = openrouteservice.Client(key="eyJvcmciOiI1YjNjZTM1OTc4NTExMTAwMDFjZjYyNDgiLCJpZCI6IjU5MjExYjU5NjgyNTRhNmM5OTZjY2Q1NzgwMzMwMDc1IiwiaCI6Im11cm11cjY0In0=")

locations = [
    [-72.093775, -36.562653],  # Bodega (Inicio)
    [-72.105695, -36.615563],  # Cliente 1
    [-72.086749, -36.624249],  # Cliente 2
    [-72.085021, -36.590386],  # Cliente 3
    [-73.07029005659592, -36.794675668452996], #Cliente 4 Lider Trebol CCP
    [-73.03562944645336, -36.793059496945226] #CLiente 5 Acuenta CCP
]

# Matriz de distancias y duraciones
matrix = client.distance_matrix(
    locations=locations,
    profile="driving-hgv",
    metrics=["duration", "distance"],
    units="m"
)

durations = matrix['durations']  # Segundos
distances = matrix['distances']  # Metros

print("Duraciones (minutos):")
for row in durations:
    print(row)
print("\nDistancias (metros):")
for row in distances:
    print(row)

# ----------------------------
# Datos para OR-Tools
# ----------------------------
def create_data_model():
    data = {}
    data['time_matrix'] = durations
    data['num_vehicles'] = 1  # Número de camiones
    data['depot'] = 0          # Índice del depósito
    # Estructura para futuras restricciones:
    data['allowed_vehicles_per_customer'] = {
        # Ejemplo: el cliente 1 solo puede ser atendido por vehículo 1
        # 1: [1]
        # Actualmente no se aplica, se deja vacío
    }
    return data

data = create_data_model()

manager = pywrapcp.RoutingIndexManager(len(data['time_matrix']),
                                       data['num_vehicles'],
                                       data['depot'])
routing = pywrapcp.RoutingModel(manager)

# ----------------------------
# Función de costo (tiempo)
# ----------------------------
def time_callback(from_index, to_index):
    from_node = manager.IndexToNode(from_index)
    to_node = manager.IndexToNode(to_index)
    return int(data['time_matrix'][from_node][to_node])

transit_callback_index = routing.RegisterTransitCallback(time_callback)
routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

# ----------------------------
# Configurar restricciones futuras
# ----------------------------
for customer_index, allowed_vehicles in data['allowed_vehicles_per_customer'].items():
    routing.SetAllowedVehiclesForIndex(
        allowed_vehicles,
        manager.NodeToIndex(customer_index)
    )

# ----------------------------
# Parámetros de búsqueda
# ----------------------------
search_parameters = pywrapcp.DefaultRoutingSearchParameters()
search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC

# ----------------------------
# Resolver
# ----------------------------
solution = routing.SolveWithParameters(search_parameters)

# ----------------------------
# Construir ruta como lista de índices
# ----------------------------
ruta_optima_indices = []
if solution:
    index = routing.Start(0)
    while not routing.IsEnd(index):
        ruta_optima_indices.append(manager.IndexToNode(index))
        index = solution.Value(routing.NextVar(index))
    ruta_optima_indices.append(manager.IndexToNode(index))  # volver al depósito

    # Imprimir ruta y tiempo total
    print("\nRuta óptima:", ruta_optima_indices)
    tiempo_total = sum(
        durations[ruta_optima_indices[i]][ruta_optima_indices[i+1]]
        for i in range(len(ruta_optima_indices)-1)
    )
    print(f"Tiempo total: {tiempo_total/60:.2f} minutos")
else:
    print("No se encontró solución")

# ----------------------------
# Visualización con Folium
# ----------------------------
m = folium.Map(location=[-36.562653, -72.093775], zoom_start=12)

for i in range(len(ruta_optima_indices)-1):
    start = locations[ruta_optima_indices[i]]
    end = locations[ruta_optima_indices[i+1]]
    route = client.directions(
        coordinates=[start, end],
        profile='driving-hgv',
        format='geojson'
    )
    folium.GeoJson(route, style_function=lambda x: {'color': 'blue', 'weight': 5}).add_to(m)

# Marcar clientes
for i, loc in enumerate(locations):
    folium.Marker([loc[1], loc[0]], popup=f"Cliente {i}").add_to(m)

m.save("ruta_calles.html")
