import openrouteservice
from openrouteservice.exceptions import ApiError
from ortools.constraint_solver import pywrapcp, routing_enums_pb2
import folium
from datetime import datetime
import configparser
from tkinter import messagebox
from db import db_execute

def get_api_key():
    """Lee la clave de API desde el archivo de configuración."""
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

def proceso_de_calculo(camion_id, clientes_seleccionados_ids, todos_los_clientes):
    """
    Calcula la ruta óptima utilizando OpenRouteService y OR-Tools.
    Devuelve un diccionario con el texto de la ruta y el archivo del mapa.
    """
    if not client:
        return {'error': "Cliente API no inicializado. Revisa tu clave de API."}
    if not camion_id or not clientes_seleccionados_ids:
        return {'error': "Selecciona un camión y al menos un cliente."}
    try:
        locs, nombres = [], []
        for c in todos_los_clientes:
            if c[0] in clientes_seleccionados_ids:
                locs.append([c[2], c[3]])
                nombres.append(c[1])
        
        locs.insert(0, [-72.093775, -36.562653]) 
        nombres.insert(0, "Bodega (Inicio)")
        
        matrix = client.distance_matrix(locations=locs, profile="driving-hgv", metrics=["duration"])
        durations = matrix['durations']

        manager = pywrapcp.RoutingIndexManager(len(durations), 1, 0)
        routing = pywrapcp.RoutingModel(manager)

        def time_callback(from_index, to_index):
            return int(durations[manager.IndexToNode(from_index)][manager.IndexToNode(to_index)])
        
        transit_callback_index = routing.RegisterTransitCallback(time_callback)
        routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
        
        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
        solution = routing.SolveWithParameters(search_parameters)

        if not solution:
            return {'error': "No se encontró una solución para la ruta."}

        ruta_indices, ruta_texto = [], "Ruta óptima:\n"
        index = routing.Start(0)
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            ruta_indices.append(node_index)
            ruta_texto += f"{len(ruta_indices)-1}. {nombres[node_index]}\n"
            index = solution.Value(routing.NextVar(index))
        
        fecha = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        clientes_str = ",".join(map(str, clientes_seleccionados_ids))
        db_execute("INSERT INTO rutas (camion_id, fecha, clientes) VALUES (?, ?, ?)", (camion_id, fecha, clientes_str))
        
        tiempo_total_segundos = solution.ObjectiveValue()
        ruta_texto += f"\nTiempo total: {tiempo_total_segundos / 60:.2f} minutos\n"
        
        ruta_coordenadas = [locs[idx] for idx in ruta_indices]
        if ruta_coordenadas:
            ruta_coordenadas.append(ruta_coordenadas[0])
        
        route_geojson = client.directions(coordinates=ruta_coordenadas, profile='driving-hgv', format='geojson')
        m = folium.Map(location=[-36.562653, -72.093775], zoom_start=9)
        folium.GeoJson(route_geojson, style_function=lambda x: {'color':'blue','weight':4}).add_to(m)
        
        for i, idx in enumerate(ruta_indices):
            loc = locs[idx]
            folium.Marker([loc[1], loc[0]], popup=f"{i}. {nombres[idx]}",
                        icon=folium.Icon(color="green" if idx == 0 else "blue")).add_to(m)
        
        map_file = "ruta_optima.html"
        m.save(map_file)
        
        return {'ruta_texto': ruta_texto, 'map_file': map_file}
    
    except ApiError as e:
        return {'error': f"Error de API: {e.args[0]}"}
    except Exception as e:
        return {'error': f"Ocurrió un error inesperado: {e}"}
