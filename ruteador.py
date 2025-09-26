import tkinter as tk
from tkinter import messagebox, ttk
import sqlite3
import openrouteservice
from openrouteservice.exceptions import ApiError
from ortools.constraint_solver import pywrapcp, routing_enums_pb2
import folium
import webbrowser
from datetime import datetime
import configparser
import threading
import re

# --- CONFIGURACIÓN Y CONSTANTES ---
DB_FILE = "rutas.db"

# Función para cargar la clave de API de forma segura desde config.ini
def get_api_key():
    """Lee la clave de API desde el archivo config.ini."""
    config = configparser.ConfigParser()
    try:
        config.read('config.ini')
        api_key = config['API']['key']
        if not api_key or api_key == 'tu_clave_aqui':
            messagebox.showerror("Error de Configuración", 
                                 "No se encontró la clave de API. "
                                 "Asegúrate de crear un archivo 'config.ini' con tu clave.")
            return None
        return api_key
    except (FileNotFoundError, KeyError):
        messagebox.showerror("Error de Configuración", 
                             "No se pudo encontrar o leer el archivo 'config.ini'. "
                             "Por favor, créalo con tu clave de API.")
        return None

# Inicializa el cliente de OpenRouteService
api_key = get_api_key()
if api_key:
    client = openrouteservice.Client(key=api_key)
else:
    # Si no hay clave, el programa no podrá funcionar.
    # Podríamos cerrar la app aquí, o dejar que falle en el cálculo.
    client = None

# --- FUNCIONES DE BASE DE DATOS ---
def db_query(query, params=()):
    """Ejecuta una consulta a la base de datos y devuelve los resultados."""
    with sqlite3.connect(DB_FILE) as conn:
        cur = conn.cursor()
        cur.execute(query, params)
        return cur.fetchall()

def db_execute(query, params=()):
    """Ejecuta una operación de escritura (INSERT, UPDATE) y devuelve el ID de la última fila."""
    with sqlite3.connect(DB_FILE) as conn:
        cur = conn.cursor()
        cur.execute(query, params)
        conn.commit()
        return cur.lastrowid

def get_camiones():
    return db_query("SELECT id, patente, tipo FROM camiones ORDER BY id")

def get_clientes():
    return db_query("SELECT id, nombre, lon, lat, tipo_camion FROM clientes")

def obtener_o_crear_camion(patente, tipo):
    """Obtiene el ID de un camión si existe, si no, lo crea."""
    result = db_query("SELECT id FROM camiones WHERE patente = ?", (patente,))
    if result:
        return result[0][0]
    else:
        return db_execute("INSERT INTO camiones (patente, tipo) VALUES (?, ?)", (patente, tipo))

def save_ruta(camion_id, clientes_ids):
    """Guarda una ruta completada en la base de datos."""
    fecha = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    clientes_str = ",".join(map(str, clientes_ids))
    db_execute("INSERT INTO rutas (camion_id, fecha, clientes) VALUES (?, ?, ?)",
               (camion_id, fecha, clientes_str))

def validar_patente(patente):
    """Valida el formato de patente (XXXX00 o XX0000)."""
    return bool(re.fullmatch(r"[A-Z]{4}\d{2}|[A-Z]{2}\d{4}", patente.upper()))


# --- LÓGICA DE CÁLCULO DE RUTA (Función principal que se ejecutará en un hilo) ---
def proceso_de_calculo(camion_id, clientes_seleccionados_ids):
    """
    Realiza todo el proceso pesado: llamadas a API, cálculo y generación de mapa.
    Devuelve un diccionario con los resultados o un mensaje de error.
    """
    if not client:
        return {'error': "Cliente API no inicializado. Revisa tu clave de API."}
    if not camion_id or not clientes_seleccionados_ids:
        return {'error': "Selecciona un camión y al menos un cliente."}

    try:
        # 1. Preparar ubicaciones
        clientes_data = get_clientes()
        locs = []
        nombres = []
        clientes_en_ruta = [] # Para guardar en la BD

        # Filtrar solo los clientes seleccionados
        for c in clientes_data:
            if c[0] in clientes_seleccionados_ids:
                locs.append([c[2], c[3]])  # [lon, lat]
                nombres.append(c[1])
                clientes_en_ruta.append(c[0])

        locs.insert(0, [-72.093775, -36.562653]) # Bodega al inicio
        nombres.insert(0, "Bodega (Inicio)")

        # 2. Obtener matriz de distancias/duraciones de la API
        matrix = client.distance_matrix(
            locations=locs, profile="driving-hgv", metrics=["duration"]
        )
        durations = matrix['durations']
        
        # 3. Configurar y resolver con OR-Tools
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

        # 4. Procesar la solución
        ruta_indices = []
        index = routing.Start(0)
        while not routing.IsEnd(index):
            ruta_indices.append(manager.IndexToNode(index))
            index = solution.Value(routing.NextVar(index))
        
        save_ruta(camion_id, clientes_en_ruta)

        # 5. Preparar texto de salida
        ruta_texto = "Ruta óptima:\n"
        for i, idx in enumerate(ruta_indices):
            ruta_texto += f"{i}. {nombres[idx]}\n"
        
        tiempo_total_segundos = solution.ObjectiveValue()
        ruta_texto += f"\nTiempo total: {tiempo_total_segundos / 60:.2f} minutos\n"

        # 6. Generar mapa con una sola llamada a la API
        ruta_coordenadas = [locs[idx] for idx in ruta_indices]
        
        # OpenRouteService requiere una parada final para cerrar el bucle si se desea
        # En este caso, no es necesario, la ruta es A->B->C
        ruta_coordenadas.append(locs[ruta_indices[-1]]) # Repetir el último para mostrarlo en el mapa

        route_geojson = client.directions(
            coordinates=ruta_coordenadas, profile='driving-hgv', format='geojson'
        )
        
        m = folium.Map(location=[-36.562653, -72.093775], zoom_start=9)
        folium.GeoJson(route_geojson, style_function=lambda x: {'color':'blue','weight':4}).add_to(m)
        
        for i, idx in enumerate(ruta_indices):
            loc = locs[idx]
            folium.Marker(
                [loc[1], loc[0]], 
                popup=f"{i}. {nombres[idx]}",
                icon=folium.Icon(color="green" if idx == 0 else "blue")
            ).add_to(m)

        map_file = "ruta_optima.html"
        m.save(map_file)

        return {'ruta_texto': ruta_texto, 'map_file': map_file}

    except ApiError as e:
        return {'error': f"Error de API: {e.args[0]}"}
    except Exception as e:
        return {'error': f"Ocurrió un error inesperado: {e}"}

# --- INTERFAZ GRÁFICA (Clase para organizar la GUI) ---
# --- INTERFAZ GRÁFICA (Clase para organizar la GUI) ---
class App:
    def __init__(self, root):
        self.root = root
        self.root.title("Ruteador de Camiones")
        self.root.geometry("800x650")

        self.setup_ui()
        self.cargar_datos_iniciales()

    def setup_ui(self):
        # Frame principal
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky="nsew")
        self.root.grid_columnconfigure(0, weight=1)
        self.root.grid_rowconfigure(0, weight=1)

        # Sección del Camión
        camion_frame = ttk.LabelFrame(main_frame, text="1. Datos del Camión", padding="10")
        camion_frame.grid(row=0, column=0, columnspan=2, sticky="ew", padx=5, pady=5)
        
        ttk.Label(camion_frame, text="Patente:").grid(row=0, column=0, sticky="w")
        self.entry_patente = ttk.Entry(camion_frame, width=20)
        self.entry_patente.grid(row=0, column=1, padx=5, pady=5)

        ttk.Label(camion_frame, text="Tipo de camión:").grid(row=1, column=0, sticky="w")
        ## CAMBIO: Se revierte la lista a los tipos originales.
        self.combo_tipo = ttk.Combobox(camion_frame, values=["Largo", "Corto", "Dolly"], state="readonly")
        self.combo_tipo.current(0)
        self.combo_tipo.grid(row=1, column=1, padx=5, pady=5)
        ## CAMBIO: La siguiente línea fue eliminada, ya no se asocia un evento al combobox.
        # self.combo_tipo.bind("<<ComboboxSelected>>", self.actualizar_lista_clientes)

        ttk.Button(camion_frame, text="Ver camiones registrados", command=self.mostrar_camiones).grid(row=0, column=2, padx=10)

        # Sección de Clientes
        clientes_frame = ttk.LabelFrame(main_frame, text="2. Selección de Clientes", padding="10")
        clientes_frame.grid(row=1, column=0, sticky="nsew", padx=5, pady=5)
        main_frame.grid_rowconfigure(1, weight=1)
        main_frame.grid_columnconfigure(0, weight=1)

        scrollbar = ttk.Scrollbar(clientes_frame, orient=tk.VERTICAL)
        self.listbox_clientes = tk.Listbox(clientes_frame, selectmode=tk.MULTIPLE, yscrollcommand=scrollbar.set, width=50, height=15)
        scrollbar.config(command=self.listbox_clientes.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.listbox_clientes.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Sección de Resultados
        salida_frame = ttk.LabelFrame(main_frame, text="3. Resultados de la Ruta", padding="10")
        salida_frame.grid(row=1, column=1, sticky="nsew", padx=5, pady=5)
        main_frame.grid_columnconfigure(1, weight=1)

        self.salida_texto = tk.Text(salida_frame, height=15, width=60, state="disabled")
        self.salida_texto.pack(fill=tk.BOTH, expand=True)
        
        # Botón para calcular
        self.btn_calcular = ttk.Button(main_frame, text="Calcular Ruta Óptima", command=self.iniciar_calculo, style="Accent.TButton")
        self.btn_calcular.grid(row=2, column=0, columnspan=2, pady=10)

        # Estilo para el botón
        style = ttk.Style()
        style.configure("Accent.TButton", font=("Helvetica", 12, "bold"))

    def cargar_datos_iniciales(self):
        """Carga la lista completa de clientes y la muestra en la Listbox."""
        ## CAMBIO: La lógica de mostrar clientes ahora está aquí directamente.
        self.todos_los_clientes = get_clientes()
        self.listbox_clientes.delete(0, tk.END)
        for cliente in self.todos_los_clientes:
            # cliente[4] es 'tipo_camion' del cliente, lo mostramos para información.
            self.listbox_clientes.insert(tk.END, f"{cliente[0]} - {cliente[1]} ({cliente[4]})")

    ## CAMBIO: El método "actualizar_lista_clientes" fue eliminado por completo.

    def mostrar_camiones(self):
        """Muestra una ventana con todos los camiones registrados."""
        top = tk.Toplevel(self.root)
        top.title("Camiones Registrados")
        top.geometry("400x300")
        
        columns = ("ID", "Patente", "Tipo")
        tree = ttk.Treeview(top, columns=columns, show="headings")
        for col in columns:
            tree.heading(col, text=col)
            tree.column(col, width=100)
        tree.pack(fill=tk.BOTH, expand=True)

        for row in get_camiones():
            tree.insert("", tk.END, values=row)

    def iniciar_calculo(self):
        """Prepara y lanza el cálculo en un hilo separado para no bloquear la GUI."""
        patente = self.entry_patente.get().strip().upper()
        if not validar_patente(patente):
            messagebox.showwarning("Entrada Inválida", "El formato de la patente no es válido (ej: AABB11 o ABCD11).")
            return
        
        tipo_camion = self.combo_tipo.get()
        ## CAMBIO: Se eliminó la comprobación de "Todos".
        
        camion_id = obtener_o_crear_camion(patente, tipo_camion)

        clientes_seleccionados_ids = []
        for idx in self.listbox_clientes.curselection():
            cliente_id_str = self.listbox_clientes.get(idx).split(" - ")[0]
            clientes_seleccionados_ids.append(int(cliente_id_str))

        if not clientes_seleccionados_ids:
            messagebox.showwarning("Selección Vacía", "Debes seleccionar al menos un cliente de la lista.")
            return

        self.btn_calcular.config(state="disabled", text="Calculando...")
        self.salida_texto.config(state="normal")
        self.salida_texto.delete("1.0", tk.END)
        self.salida_texto.insert(tk.END, "Procesando la ruta, por favor espera...")
        self.salida_texto.config(state="disabled")

        thread = threading.Thread(
            target=self.ejecutar_y_actualizar_gui, 
            args=(camion_id, clientes_seleccionados_ids)
        )
        thread.daemon = True
        thread.start()

    def ejecutar_y_actualizar_gui(self, camion_id, clientes_ids):
        """Función que llama al proceso pesado y luego programa la actualización de la GUI."""
        resultado = proceso_de_calculo(camion_id, clientes_ids)
        self.root.after(0, self.actualizar_gui_con_resultado, resultado)
    
    def actualizar_gui_con_resultado(self, resultado):
        """Actualiza la GUI con el resultado del cálculo. Se ejecuta en el hilo principal."""
        self.salida_texto.config(state="normal")
        self.salida_texto.delete("1.0", tk.END)

        if 'error' in resultado:
            messagebox.showerror("Error en el Cálculo", resultado['error'])
            self.salida_texto.insert(tk.END, f"Error: {resultado['error']}")
        else:
            self.salida_texto.insert(tk.END, resultado['ruta_texto'])
            try:
                webbrowser.open(resultado['map_file'])
            except Exception as e:
                messagebox.showwarning("Advertencia", f"No se pudo abrir el mapa automáticamente: {e}")

        self.salida_texto.config(state="disabled")
        self.btn_calcular.config(state="normal", text="Calcular Ruta Óptima")
# --- PUNTO DE ENTRADA PRINCIPAL ---
if __name__ == "__main__":
    if not api_key:
        # Si no hay clave, no iniciar la GUI. El mensaje de error ya se mostró.
        print("Error: No se puede iniciar la aplicación sin una clave de API válida.")
    else:
        root = tk.Tk()
        app = App(root)
        root.mainloop()