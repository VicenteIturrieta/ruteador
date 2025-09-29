import tkinter as tk
from tkinter import messagebox, ttk, simpledialog
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
import sv_ttk 

# --- CONFIGURACIÓN Y CONSTANTES ---
DB_FILE = "rutas.db"

def get_api_key():
    """Lee la clave de API de forma segura desde config.ini."""
    config = configparser.ConfigParser()
    try:
        config.read('config.ini')
        api_key = config['API']['key']
        if not api_key or api_key == 'tu_clave_aqui':
            messagebox.showerror("Error de Configuración", "No se encontró la clave de API en 'config.ini'.")
            return None
        return api_key
    except (FileNotFoundError, KeyError):
        messagebox.showerror("Error de Configuración", "No se pudo encontrar o leer el archivo 'config.ini'.")
        return None

api_key = get_api_key()
client = openrouteservice.Client(key=api_key) if api_key else None

# --- FUNCIONES DE BASE DE DATOS ---
def db_query(query, params=()):
    """Ejecuta una consulta y devuelve los resultados."""
    with sqlite3.connect(DB_FILE) as conn:
        return conn.cursor().execute(query, params).fetchall()

def db_execute(query, params=()):
    """Ejecuta una operación de escritura (INSERT, UPDATE, DELETE)."""
    with sqlite3.connect(DB_FILE) as conn:
        conn.cursor().execute(query, params)
        conn.commit()

def validar_patente(patente):
    """Valida el formato de patente (XXXX00 o XX0000)."""
    return bool(re.fullmatch(r"[A-Z]{4}\d{2}|[A-Z]{2}\d{4}", patente.upper()))

# --- LÓGICA DE CÁLCULO DE RUTA (Sin cambios) ---
def proceso_de_calculo(camion_id, clientes_seleccionados_ids, todos_los_clientes):
    if not client:
        return {'error': "Cliente API no inicializado. Revisa tu clave de API."}
    if not camion_id or not clientes_seleccionados_ids:
        return {'error': "Selecciona un camión y al menos un cliente."}

    try:
        locs = []
        nombres = []
        for c in todos_los_clientes:
            if c[0] in clientes_seleccionados_ids:
                locs.append([c[2], c[3]])
                nombres.append(c[1])

        locs.insert(0, [-72.093775, -36.562653]) # Bodega
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
            ruta_coordenadas.append(ruta_coordenadas[-1])

        route_geojson = client.directions(coordinates=ruta_coordenadas, profile='driving-hgv', format='geojson')
        m = folium.Map(location=[-72.093775, -36.562653], zoom_start=9)
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

# --- INTERFAZ GRÁFICA ---
class App:
    def __init__(self, root):
        self.root = root
        self.root.title("Ruteador de Camiones v0.2")
        self.root.geometry("850x700")
        
        self.style = ttk.Style()
        self.style.configure(".", font=("Segoe UI", 10))

        self.all_clientes_data = []
        self.setup_ui()
        self.refrescar_datos_locales_y_ui()

    # --- 1. CONFIGURACIÓN DE LA INTERFAZ ---
    def setup_ui(self):
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky="nsew")
        self.root.grid_columnconfigure(0, weight=1)
        self.root.grid_rowconfigure(0, weight=1)
        main_frame.grid_rowconfigure(2, weight=1)

        # --- SECCIÓN 0: FILTROS (CD Y ZONA) ---
        cd_frame = ttk.LabelFrame(main_frame, text="0. Filtros de Búsqueda", padding="10")
        cd_frame.grid(row=0, column=0, columnspan=2, sticky="ew", padx=5, pady=5)
        cd_frame.grid_columnconfigure(1, weight=1)

        ttk.Label(cd_frame, text="Centro de Distribución:").grid(row=0, column=0, sticky="w", padx=5, pady=2)
        self.combo_cd = ttk.Combobox(cd_frame, values=[], state="readonly")
        self.combo_cd.grid(row=0, column=1, sticky="ew", padx=5)
        self.combo_cd.bind("<<ComboboxSelected>>", self.cargar_zonas_por_cd)

        ttk.Label(cd_frame, text="Zona:").grid(row=1, column=0, sticky="w", padx=5, pady=2)
        self.combo_zona = ttk.Combobox(cd_frame, values=[], state="readonly")
        self.combo_zona.grid(row=1, column=1, sticky="ew", padx=5)
        self.combo_zona.bind("<<ComboboxSelected>>", self.cargar_clientes_por_zona)

        botones_cd_zona = ttk.Frame(cd_frame)
        botones_cd_zona.grid(row=0, column=2, rowspan=2, padx=10)
        ttk.Button(botones_cd_zona, text="Agregar CD", command=self.agregar_cd).pack(fill='x')
        ttk.Button(botones_cd_zona, text="Agregar Zona", command=self.agregar_zona).pack(fill='x', pady=2)
        ttk.Button(botones_cd_zona, text="Editar Zona", command=self.editar_zona).pack(fill='x')
        ttk.Button(botones_cd_zona, text="Eliminar Zona", command=self.eliminar_zona).pack(fill='x', pady=2)

        # --- SECCIÓN 1: DATOS DEL CAMIÓN ---
        camion_frame = ttk.LabelFrame(main_frame, text="1. Datos del Camión", padding="10")
        camion_frame.grid(row=1, column=0, columnspan=2, sticky="ew", padx=5, pady=5)
        ttk.Label(camion_frame, text="Patente:").grid(row=0, column=0, sticky="w", pady=2)
        self.entry_patente = ttk.Entry(camion_frame, width=20)
        self.entry_patente.grid(row=0, column=1, padx=5)
        ttk.Label(camion_frame, text="Tipo:").grid(row=1, column=0, sticky="w", pady=2)
        self.combo_tipo = ttk.Combobox(camion_frame, values=["Largo", "Corto", "Dolly"], state="readonly")
        self.combo_tipo.current(0)
        self.combo_tipo.grid(row=1, column=1, padx=5)
        ttk.Button(camion_frame, text="Ver/Editar Camiones", command=self.mostrar_camiones).grid(row=0, column=2, rowspan=2, padx=10)
        
        # --- SECCIÓN 2: CLIENTES ---
        clientes_frame = ttk.LabelFrame(main_frame, text="2. Clientes de la Zona", padding="10")
        clientes_frame.grid(row=2, column=0, sticky="nsew", padx=5, pady=5)
        clientes_frame.grid_rowconfigure(0, weight=1)
        clientes_frame.grid_columnconfigure(1, weight=1)

        botones_clientes = ttk.Frame(clientes_frame)
        botones_clientes.grid(row=0, column=0, sticky="ns", padx=5)
        ttk.Button(botones_clientes, text="Agregar Cliente", command=self.agregar_cliente).pack(fill='x', pady=2)
        ttk.Button(botones_clientes, text="Editar Cliente", command=self.editar_cliente).pack(fill='x')
        ttk.Button(botones_clientes, text="Eliminar Cliente", command=self.eliminar_cliente).pack(fill='x', pady=2)

        listbox_frame = ttk.Frame(clientes_frame)
        listbox_frame.grid(row=0, column=1, sticky="nsew")
        listbox_frame.grid_rowconfigure(0, weight=1)
        listbox_frame.grid_columnconfigure(0, weight=1)
        scrollbar = ttk.Scrollbar(listbox_frame, orient=tk.VERTICAL)
        self.listbox_clientes = tk.Listbox(listbox_frame, selectmode=tk.MULTIPLE, yscrollcommand=scrollbar.set, background="#fdfdfd", foreground="#1e1e1e", borderwidth=0, highlightthickness=0)
        scrollbar.config(command=self.listbox_clientes.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.listbox_clientes.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # --- SECCIÓN 3: RESULTADOS ---
        salida_frame = ttk.LabelFrame(main_frame, text="3. Resultados de la Ruta", padding="10")
        salida_frame.grid(row=2, column=1, sticky="nsew", padx=5, pady=5)
        salida_frame.grid_rowconfigure(0, weight=1)
        salida_frame.grid_columnconfigure(0, weight=1)
        self.salida_texto = tk.Text(salida_frame, height=15, width=60, state="disabled", wrap="word", background="#fdfdfd", foreground="#1e1e1e", borderwidth=0, highlightthickness=0)
        self.salida_texto.grid(row=0, column=0, sticky="nsew")

        # --- Botón de Calcular ---
        self.style.configure("Accent.TButton", font=("Segoe UI", 12, "bold"))
        self.btn_calcular = ttk.Button(main_frame, text="Calcular Ruta Óptima", command=self.iniciar_calculo, style="Accent.TButton")
        self.btn_calcular.grid(row=3, column=0, columnspan=2, pady=10)

    # --- 2. GESTIÓN DE DATOS Y ACTUALIZACIÓN DE UI (Sin cambios) ---
    def refrescar_datos_locales_y_ui(self):
        self.all_clientes_data = db_query("SELECT id, nombre, lon, lat, tipo_camion, cd, zona FROM clientes")
        self.cargar_cds()

    def cargar_cds(self):
        cds = sorted(set([c[5] for c in self.all_clientes_data if c[5]]))
        self.combo_cd['values'] = cds
        if cds: self.combo_cd.set(cds[0])
        else: self.combo_cd.set("")
        self.cargar_zonas_por_cd()

    def cargar_zonas_por_cd(self, event=None):
        cd = self.combo_cd.get()
        zonas = sorted(set([c[6] for c in self.all_clientes_data if c[5] == cd and c[6]]))
        self.combo_zona['values'] = zonas
        if zonas: self.combo_zona.set(zonas[0])
        else: self.combo_zona.set("")
        self.cargar_clientes_por_zona()

    def cargar_clientes_por_zona(self, event=None):
        cd, zona = self.combo_cd.get(), self.combo_zona.get()
        self.listbox_clientes.delete(0, tk.END)
        clientes_filtrados = [c for c in self.all_clientes_data if c[5] == cd and c[6] == zona]
        for c in clientes_filtrados:
            self.listbox_clientes.insert(tk.END, f"{c[0]} - {c[1]}")

    # --- 3. MÉTODOS CRUD (Sin cambios) ---
    def agregar_cd(self):
        nuevo_cd = simpledialog.askstring("Agregar CD", "Nombre del nuevo Centro de Distribución:")
        if nuevo_cd:
            messagebox.showinfo("Paso Siguiente", f"CD '{nuevo_cd}' listo. Ahora agrega una zona y un cliente para inicializarlo.")
            cds_actuales = list(self.combo_cd['values'])
            if nuevo_cd not in cds_actuales:
                cds_actuales.append(nuevo_cd)
                self.combo_cd['values'] = sorted(cds_actuales)
            self.combo_cd.set(nuevo_cd)
            self.combo_zona['values'] = []
            self.combo_zona.set("")
            self.listbox_clientes.delete(0, tk.END)

    def agregar_zona(self):
        cd = self.combo_cd.get()
        if not cd:
            messagebox.showwarning("Atención", "Selecciona un CD primero.")
            return
        nueva_zona = simpledialog.askstring("Agregar Zona", f"Nombre de la nueva zona para el CD '{cd}':")
        if nueva_zona:
             messagebox.showinfo("Paso Siguiente", f"Zona '{nueva_zona}' lista. Ahora agrega un cliente para guardarla.")
             self.combo_zona.set(nueva_zona)
             self.listbox_clientes.delete(0, tk.END)
    
    def editar_zona(self):
        cd, zona_actual = self.combo_cd.get(), self.combo_zona.get()
        if not cd or not zona_actual:
            messagebox.showwarning("Atención", "Selecciona un CD y una Zona para editar.")
            return
        nuevo_nombre = simpledialog.askstring("Editar Zona", f"Nuevo nombre para la zona '{zona_actual}':", initialvalue=zona_actual)
        if nuevo_nombre and nuevo_nombre != zona_actual:
            if messagebox.askyesno("Confirmar", f"¿Seguro de renombrar la zona '{zona_actual}' a '{nuevo_nombre}'?"):
                db_execute("UPDATE clientes SET zona=? WHERE cd=? AND zona=?", (nuevo_nombre, cd, zona_actual))
                messagebox.showinfo("Éxito", "La zona ha sido renombrada.")
                self.refrescar_datos_locales_y_ui()

    def eliminar_zona(self):
        cd, zona = self.combo_cd.get(), self.combo_zona.get()
        if not cd or not zona:
            messagebox.showwarning("Atención", "Selecciona un CD y una Zona para eliminar.")
            return
        if messagebox.askyesno("Confirmar Eliminación", f"¿SEGURO que deseas eliminar la zona '{zona}' y TODOS sus clientes?"):
            db_execute("DELETE FROM clientes WHERE cd=? AND zona=?", (cd, zona))
            messagebox.showinfo("Éxito", "La zona y sus clientes han sido eliminados.")
            self.refrescar_datos_locales_y_ui()

    def agregar_cliente(self):
        cd, zona = self.combo_cd.get(), self.combo_zona.get()
        if not cd or not zona:
            messagebox.showwarning("Atención", "Debes seleccionar un CD y una Zona.")
            return
        top = tk.Toplevel(self.root)
        top.title(f"Agregar Cliente a {cd}/{zona}")
        top.geometry("300x300")
        ttk.Label(top, text="Nombre:").pack(pady=5); entry_nombre = ttk.Entry(top, width=30); entry_nombre.pack()
        ttk.Label(top, text="Longitud (Lon):").pack(pady=5); entry_lon = ttk.Entry(top, width=30); entry_lon.pack()
        ttk.Label(top, text="Latitud (Lat):").pack(pady=5); entry_lat = ttk.Entry(top, width=30); entry_lat.pack()
        ttk.Label(top, text="Tipo de camión req.:").pack(pady=5); combo_tipo = ttk.Combobox(top, values=["Largo", "Corto", "Dolly"], state="readonly"); combo_tipo.current(0); combo_tipo.pack()
        def guardar():
            nombre, lon_str, lat_str, tipo = entry_nombre.get().strip(), entry_lon.get().strip(), entry_lat.get().strip(), combo_tipo.get()
            if not all([nombre, lon_str, lat_str]):
                messagebox.showwarning("Campos Vacíos", "Todos los campos son obligatorios.", parent=top); return
            try:
                lon, lat = float(lon_str), float(lat_str)
                db_execute("INSERT INTO clientes (nombre, lon, lat, tipo_camion, cd, zona) VALUES (?, ?, ?, ?, ?, ?)", (nombre, lon, lat, tipo, cd, zona))
                messagebox.showinfo("Éxito", "Cliente agregado.", parent=top)
                top.destroy()
                self.refrescar_datos_locales_y_ui()
            except ValueError:
                messagebox.showerror("Error de Formato", "Longitud y Latitud deben ser números.", parent=top)
        ttk.Button(top, text="Guardar Cliente", command=guardar).pack(pady=10)

    def editar_cliente(self):
        seleccion = self.listbox_clientes.curselection()
        if not seleccion:
            messagebox.showwarning("Atención", "Selecciona un cliente para editar."); return
        cliente_id = int(self.listbox_clientes.get(seleccion[0]).split(" - ")[0])
        cliente = next((c for c in self.all_clientes_data if c[0] == cliente_id), None)
        if not cliente: return
        top = tk.Toplevel(self.root); top.title(f"Editar Cliente: {cliente[1]}"); top.geometry("300x300")
        ttk.Label(top, text="Nombre:").pack(pady=5); entry_nombre = ttk.Entry(top, width=30); entry_nombre.insert(0, cliente[1]); entry_nombre.pack()
        ttk.Label(top, text="Longitud (Lon):").pack(pady=5); entry_lon = ttk.Entry(top, width=30); entry_lon.insert(0, cliente[2]); entry_lon.pack()
        ttk.Label(top, text="Latitud (Lat):").pack(pady=5); entry_lat = ttk.Entry(top, width=30); entry_lat.insert(0, cliente[3]); entry_lat.pack()
        ttk.Label(top, text="Tipo de camión req.:").pack(pady=5); combo_tipo = ttk.Combobox(top, values=["Largo", "Corto", "Dolly"], state="readonly"); combo_tipo.set(cliente[4]); combo_tipo.pack()
        def guardar():
            nombre, lon_str, lat_str, tipo = entry_nombre.get().strip(), entry_lon.get().strip(), entry_lat.get().strip(), combo_tipo.get()
            if not all([nombre, lon_str, lat_str]):
                messagebox.showwarning("Campos Vacíos", "Todos los campos son obligatorios.", parent=top); return
            if messagebox.askyesno("Confirmar", "¿Guardar los cambios en este cliente?", parent=top):
                try:
                    lon, lat = float(lon_str), float(lat_str)
                    db_execute("UPDATE clientes SET nombre=?, lon=?, lat=?, tipo_camion=? WHERE id=?", (nombre, lon, lat, tipo, cliente_id))
                    messagebox.showinfo("Éxito", "Cliente actualizado.", parent=top)
                    top.destroy()
                    self.refrescar_datos_locales_y_ui()
                except ValueError:
                    messagebox.showerror("Error de Formato", "Longitud y Latitud deben ser números.", parent=top)
        ttk.Button(top, text="Guardar Cambios", command=guardar).pack(pady=10)

    def eliminar_cliente(self):
        seleccion = self.listbox_clientes.curselection()
        if not seleccion:
            messagebox.showwarning("Atención", "Selecciona al menos un cliente para eliminar."); return
        if messagebox.askyesno("Confirmar Eliminación", f"¿Seguro que deseas eliminar {len(seleccion)} cliente(s)?"):
            for idx in reversed(seleccion):
                cliente_id = int(self.listbox_clientes.get(idx).split(" - ")[0])
                db_execute("DELETE FROM clientes WHERE id = ?", (cliente_id,))
            messagebox.showinfo("Éxito", "Cliente(s) eliminado(s).")
            self.refrescar_datos_locales_y_ui()

    def mostrar_camiones(self):
        top = tk.Toplevel(self.root); top.title("Gestión de Camiones"); top.geometry("450x300")
        tree = ttk.Treeview(top, columns=("ID", "Patente", "Tipo"), show="headings")
        tree.heading("ID", text="ID"); tree.column("ID", width=50)
        tree.heading("Patente", text="Patente"); tree.column("Patente", width=150)
        tree.heading("Tipo", text="Tipo"); tree.column("Tipo", width=150)
        tree.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        def refrescar_tabla():
            tree.delete(*tree.get_children())
            for row in db_query("SELECT id, patente, tipo FROM camiones ORDER BY id"):
                tree.insert("", tk.END, values=row)
        def editar():
            if not tree.selection(): return
            item = tree.item(tree.selection()[0])
            camion_id, patente, tipo = item['values']
            editor = tk.Toplevel(top); editor.title("Editar Camión")
            ttk.Label(editor, text="Patente:").pack(pady=5); entry_patente = ttk.Entry(editor); entry_patente.insert(0, patente); entry_patente.pack()
            ttk.Label(editor, text="Tipo:").pack(pady=5); combo_tipo = ttk.Combobox(editor, values=["Largo", "Corto", "Dolly"], state="readonly"); combo_tipo.set(tipo); combo_tipo.pack()
            def guardar():
                nueva_patente, nuevo_tipo = entry_patente.get().strip().upper(), combo_tipo.get()
                if not validar_patente(nueva_patente):
                    messagebox.showerror("Error", "Formato de patente inválido.", parent=editor); return
                if messagebox.askyesno("Confirmar", "¿Guardar cambios en el camión?", parent=editor):
                    db_execute("UPDATE camiones SET patente=?, tipo=? WHERE id=?", (nueva_patente, nuevo_tipo, camion_id))
                    messagebox.showinfo("Éxito", "Camión actualizado.", parent=editor)
                    editor.destroy(); refrescar_tabla()
            ttk.Button(editor, text="Guardar", command=guardar).pack(pady=10)
        def eliminar():
            if not tree.selection(): return
            if messagebox.askyesno("Confirmar", "¿Eliminar el camión seleccionado?"):
                item = tree.item(tree.selection()[0])
                db_execute("DELETE FROM camiones WHERE id = ?", (item['values'][0],))
                messagebox.showinfo("Éxito", "Camión eliminado."); refrescar_tabla()
        botones_frame = ttk.Frame(top); botones_frame.pack(pady=5)
        ttk.Button(botones_frame, text="Editar Seleccionado", command=editar).pack(side=tk.LEFT, padx=5)
        ttk.Button(botones_frame, text="Eliminar Seleccionado", command=eliminar).pack(side=tk.LEFT, padx=5)
        refrescar_tabla()

    # --- 4. LÓGICA PRINCIPAL DE LA APLICACIÓN (Sin cambios) ---
    def iniciar_calculo(self):
        patente = self.entry_patente.get().strip().upper()
        if not validar_patente(patente):
            messagebox.showwarning("Entrada Inválida", "Formato de patente no válido."); return
        
        tipo_camion = self.combo_tipo.get()
        result = db_query("SELECT id FROM camiones WHERE patente = ?", (patente,))
        if result:
            camion_id = result[0][0]
        else:
            db_execute("INSERT INTO camiones (patente, tipo) VALUES (?, ?)", (patente, tipo_camion))
            camion_id = db_query("SELECT id FROM camiones WHERE patente = ?", (patente,))[0][0]

        seleccion = self.listbox_clientes.curselection()
        if not seleccion:
            messagebox.showwarning("Selección Vacía", "Debes seleccionar al menos un cliente."); return
        clientes_ids = [int(self.listbox_clientes.get(i).split(" - ")[0]) for i in seleccion]

        self.btn_calcular.config(state="disabled", text="Calculando...")
        self.salida_texto.config(state="normal"); self.salida_texto.delete("1.0", tk.END)
        self.salida_texto.insert(tk.END, "Procesando la ruta, por favor espera...")
        self.salida_texto.config(state="disabled")

        thread = threading.Thread(target=self.ejecutar_y_actualizar_gui, args=(camion_id, clientes_ids))
        thread.daemon = True
        thread.start()

    def ejecutar_y_actualizar_gui(self, camion_id, clientes_ids):
        resultado = proceso_de_calculo(camion_id, clientes_ids, self.all_clientes_data)
        self.root.after(0, self.actualizar_gui_con_resultado, resultado)
    
    def actualizar_gui_con_resultado(self, resultado):
        self.salida_texto.config(state="normal")
        self.salida_texto.delete("1.0", tk.END)
        if 'error' in resultado:
            messagebox.showerror("Error en el Cálculo", resultado['error'])
            self.salida_texto.insert(tk.END, f"Error: {resultado['error']}")
        else:
            self.salida_texto.insert(tk.END, resultado['ruta_texto'])
            if messagebox.askyesno("Ruta Calculada", "La ruta se ha calculado. ¿Deseas ver el mapa?"):
                try:
                    webbrowser.open(resultado['map_file'])
                except Exception as e:
                    messagebox.showwarning("Advertencia", f"No se pudo abrir el mapa: {e}")
        self.salida_texto.config(state="disabled")
        self.btn_calcular.config(state="normal", text="Calcular Ruta Óptima")

# --- PUNTO DE ENTRADA PRINCIPAL ---
if __name__ == "__main__":
    if not client:
        print("Error: No se puede iniciar la aplicación sin una clave de API válida en config.ini")
    else:
        root = tk.Tk()
        
        sv_ttk.set_theme("light") #light o dark

        app = App(root)
        root.mainloop()