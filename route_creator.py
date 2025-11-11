import openrouteservice
from openrouteservice.exceptions import ApiError
from ortools.constraint_solver import pywrapcp, routing_enums_pb2
import folium
from datetime import datetime
import configparser
from tkinter import messagebox
from db import guardar_ruta

def get_api_key():
    config = configparser.ConfigParser()
    try:
        config.read('config.ini')
        api_key = config['API']['key']
        if not api_key or api_key == 'clave api':
            messagebox.showerror("Error de Configuración", "No se encontró la clave de API en 'config.ini'.")
            return None
        return api_key
    except (FileNotFoundError, KeyError):
        messagebox.showerror("Error de Configuración", "No se pudo encontrar o leer el archivo 'config.ini'.")
        return None

api_key = get_api_key()
client = openrouteservice.Client(key=api_key) if api_key else None

def proceso_de_calculo(camion_id, clientes_seleccionados_ids, todos_los_clientes, start_coords):
    """
    Calcula la ruta ÓPTIMA (TSP) usando OR-Tools y la matriz de ORS.
    """
    if not client:
        return {'error': "Cliente API no inicializado. Revisa tu clave de API."}
    if not camion_id or not clientes_seleccionados_ids:
        return {'error': "Selecciona un camión y al menos un cliente."}
    
    try:
        # --- PASO 1: RECOLECTAR COORDENADAS (LOCS) ---
        # El orden es [Lon, Lat]
        locs = []
        nombres = []
        
        # Añadir Bodega PRIMERO (índice 0)
        locs.append(start_coords)
        nombres.append("Bodega (Inicio)")

        for c in todos_los_clientes:
            # c[0] = id, c[1] = nombre, c[2] = lon, c[3] = lat, c[7] = dest_id
            if c[0] in clientes_seleccionados_ids:
                if c[2] is None or c[3] is None:
                    print(f"Omitiendo cliente ID {c[0]} por falta de coordenadas.")
                    continue
                
                locs.append([c[2], c[3]]) # [Lon, Lat]
                display_name = c[1] if c[1] else f"Local #{c[7]}"
                nombres.append(display_name)

        if len(locs) < 2:
            return {'error': "No hay suficientes puntos (bodega + 1 cliente) para crear una ruta."}

        # --- PASO 2: OBTENER MATRIZ DE TIEMPOS DE ORS ---
        matrix_response = client.distance_matrix(
            locations=locs,
            profile="driving-hgv",
            metrics=["duration"]
        )
        durations = matrix_response['durations'] # Matriz de tiempos en segundos

        # --- PASO 3: CONFIGURAR OR-TOOLS (TSP) ---
        num_locations = len(locs)
        num_vehicles = 1
        depot_index = 0 # El índice 0 es nuestra Bodega

        manager = pywrapcp.RoutingIndexManager(num_locations, num_vehicles, depot_index)
        routing = pywrapcp.RoutingModel(manager)

        # --- PASO 4: DEFINIR COSTO (CALLBACK DE TIEMPO) ---
        def time_callback(from_index, to_index):
            from_node = manager.IndexToNode(from_index)
            to_node = manager.IndexToNode(to_index)
            return int(durations[from_node][to_node])

        transit_callback_index = routing.RegisterTransitCallback(time_callback)
        routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

        # --- PASO 5: RESOLVER EL PROBLEMA ---
        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
        )
        solution = routing.SolveWithParameters(search_parameters)

        if not solution:
            return {'error': "No se encontró una solución de ruta óptima."}

        # --- PASO 6: EXTRAER LA RUTA ÓPTIMA ---
        ruta_indices_optimos = [] # Índices en el orden óptimo
        ruta_texto = "Ruta óptima:\n"
        index = routing.Start(0)
        
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index) # El índice original de 'locs'
            ruta_indices_optimos.append(node_index)
            index = solution.Value(routing.NextVar(index))
        
        ruta_coordenadas_optima = [locs[idx] for idx in ruta_indices_optimos]
        # Añadir el regreso a la bodega para cerrar el bucle del mapa
        ruta_coordenadas_optima.append(start_coords) 

        # --- PASO 7: GUARDAR HISTORIAL Y PREPARAR SALIDA ---
        fecha = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        clientes_str = ",".join(map(str, clientes_seleccionados_ids))
        guardar_ruta(camion_id, fecha, clientes_str)

        total_tiempo_segundos = solution.ObjectiveValue()
        
        for i, idx in enumerate(ruta_indices_optimos):
            ruta_texto += f"{i}. {nombres[idx]}\n"
        ruta_texto += f"\nTiempo total de viaje: {total_tiempo_segundos / 60:.2f} minutos"

        # --- PASO 8: DIBUJAR MAPA CON LA RUTA ÓPTIMA ---
        route_geojson = client.directions(
            coordinates=ruta_coordenadas_optima,
            profile='driving-hgv',
            format='geojson',
            instructions=False
        )
        
        # folium.Map usa [Lat, Lon], pero 'start_coords' es [Lon, Lat]
        map_center_lat_lon = [start_coords[1], start_coords[0]]
        m = folium.Map(location=map_center_lat_lon, zoom_start=9)
        
        folium.GeoJson(route_geojson['features'][0]['geometry'], style_function=lambda x: {'color':'blue','weight':4}).add_to(m)

        for i, idx in enumerate(ruta_indices_optimos):
            loc = locs[idx] # loc es [Lon, Lat]
            folium.Marker(
                [loc[1], loc[0]], # Marker usa [Lat, Lon]
                popup=f"{i}. {nombres[idx]}",
                icon=folium.Icon(color="green" if idx == 0 else "blue")
            ).add_to(m)

        map_file = "ruta_optima.html"
        m.save(map_file)
        
        return {'ruta_texto': ruta_texto, 'map_file': map_file}

    except ApiError as e:
        error_body = e.args[0]
        if 'error' in error_body:
            return {'error': f"Error de API (ORS): {error_body['error'].get('message', 'Error desconocido')}"}
        return {'error': f"Error de API: {e.args[0]}"}
    except Exception as e:
        return {'error': f"Ocurrió un error inesperado: {e}"}
    