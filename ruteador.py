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
import pandas as pd
from tkinter import filedialog

DB_FILE = "rutas.db"

def db_query(query, params=()):
    with sqlite3.connect(DB_FILE) as conn:
        return conn.cursor().execute(query, params).fetchall()
def db_execute(query, params=()):
    with sqlite3.connect(DB_FILE) as conn:
        conn.cursor().execute(query, params)
        conn.commit()

def inicializar_tabla_usuarios():
    db_execute("""
        CREATE TABLE IF NOT EXISTS usuarios (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            usuario TEXT UNIQUE NOT NULL,
            password TEXT NOT NULL,
            rol TEXT CHECK(rol IN ('admin','usuario')) NOT NULL
        )
    """)
    admin_existe = db_query("SELECT * FROM usuarios WHERE usuario = 'admin'")
    if not admin_existe:
        db_execute("INSERT INTO usuarios (usuario, password, rol) VALUES (?, ?, ?)",
                   ('admin', 'admin', 'admin'))

inicializar_tabla_usuarios()

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

def validar_patente(patente):
    return bool(re.fullmatch(r"[A-Z]{4}\d{2}|[A-Z]{2}\d{4}", patente.upper()))

def procesar_despachos_del_dia():
    filepath = filedialog.askopenfilename(
        title="Selecciona el archivo Excel de despachos",
        filetypes=(("Archivos Excel", "*.xlsx"), ("Todos los archivos", "*.*"))
    )
    if not filepath:
        return None, "No se seleccionó ningún archivo."
    try:
        df = pd.read_excel(filepath, sheet_name="Shipping Temuco")
        df_listo = df[df['Status'] == 'PICKED'].copy()
        if df_listo.empty:
            return {}, "No se encontraron despachos con estado 'PICKED'."
        df_listo['Destination #'] = df_listo['Destination'].str.split('|').str[-1].str.strip()
        df_listo.dropna(subset=['Destination #'], inplace=True)
        df_listo = df_listo[df_listo['Destination #'] != '']
        df_listo['Destination #'] = df_listo['Destination #'].astype(int)
        df_listo['Case Count'] = pd.to_numeric(df_listo['Case Count'], errors='coerce').fillna(0)
        despachos = df_listo.groupby('Destination #')['Case Count'].sum()
        despachos_dict = despachos.to_dict()
        return despachos_dict, f"Se procesaron {len(despachos_dict)} clientes con un total de {int(sum(despachos_dict.values()))} cajas."
    except KeyError as e:
        return None, f"Error: No se encontró la columna requerida en el Excel: {e}. Revisa el formato."
    except Exception as e:
        return None, f"Error al procesar el archivo Excel: {e}"

def proceso_de_calculo(camion_id, clientes_seleccionados_ids, todos_los_clientes):
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
            locs.insert(0, [-72.093775, -36.562653]) # Bodega, ver como cambiar según CD donde se implemente
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
            map_file = "ruta_historica.html"
            m.save(map_file)
            return {'ruta_texto': ruta_texto, 'map_file': map_file}
        except ApiError as e:
            return {'error': f"Error de API: {e.args[0]}"}
        except Exception as e:
            return {'error': f"Ocurrió un error inesperado: {e}"}

class App:
    def __init__(self, root, rol):
        self.rol = rol
        self.root = root
        self.root.title("Ruteador de Camiones v0.3")
        self.root.geometry("850x700")
        self.style = ttk.Style()
        self.style.configure(".", font=("Segoe UI", 10))
        self.all_clientes_data = []
        self.setup_ui()
        self.refrescar_datos_locales_y_ui()

    def setup_ui(self):

        if self.rol != "admin":
            for widget in [self.btn_calcular]:
                widget.config(state="normal")
            for child in self.root.winfo_children():
                pass
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky="nsew")
        self.root.grid_columnconfigure(0, weight=1)
        self.root.grid_rowconfigure(0, weight=1)
        main_frame.grid_columnconfigure(0, weight=1)
        main_frame.grid_columnconfigure(1, weight=0) 
        main_frame.grid_rowconfigure(1, weight=1)
        top_frame = ttk.Frame(main_frame)
        top_frame.grid(row=0, column=0, columnspan=2, sticky="ew", padx=5, pady=5)
        top_frame.grid_columnconfigure(1, weight=1)
        self.btn_cargar_excel = ttk.Button(top_frame, text="1. Cargar Despachos del Día (Excel)", command=self.cargar_y_filtrar_despachos, style="Accent.TButton")
        self.btn_cargar_excel.grid(row=0, column=0, padx=5, pady=5, sticky="ew")
        camion_frame = ttk.LabelFrame(top_frame, text="2. Datos del Camión", padding="10")
        camion_frame.grid(row=0, column=1, padx=5, pady=5, sticky="ew")
        ttk.Label(camion_frame, text="Patente:").pack(side=tk.LEFT, padx=5)
        self.entry_patente = ttk.Entry(camion_frame, width=15)
        self.entry_patente.pack(side=tk.LEFT, padx=5)
        ttk.Label(camion_frame, text="Tipo:").pack(side=tk.LEFT, padx=5)
        self.combo_tipo = ttk.Combobox(camion_frame, values=["Largo", "Corto", "Dolly"], state="readonly", width=10)
        self.combo_tipo.current(0)
        self.combo_tipo.pack(side=tk.LEFT, padx=5)
        gestion_frame = ttk.Frame(main_frame)
        gestion_frame.grid(row=0, column=1, rowspan=3, sticky="ns", padx=5, pady=5)
        clientes_gestion_frame = ttk.LabelFrame(gestion_frame, text="Gestión de Clientes", padding="10")
        clientes_gestion_frame.pack(fill="x", pady=5)
        ttk.Button(clientes_gestion_frame, text="Agregar Cliente", command=self.agregar_cliente).pack(fill='x', pady=2)
        ttk.Button(clientes_gestion_frame, text="Editar Cliente", command=self.editar_cliente).pack(fill='x')
        ttk.Button(clientes_gestion_frame, text="Eliminar Cliente", command=self.eliminar_cliente).pack(fill='x', pady=2)
        otros_gestion_frame = ttk.LabelFrame(gestion_frame, text="Gestión General", padding="10")
        otros_gestion_frame.pack(fill="x", pady=5)
        ttk.Button(otros_gestion_frame, text="Ver/Editar Camiones", command=self.mostrar_camiones).pack(fill="x", pady=2)
        ttk.Button(otros_gestion_frame, text="Historial de Rutas", command=self.mostrar_historial_rutas).pack(fill="x")
        filtros_gestion_frame = ttk.LabelFrame(gestion_frame, text="Filtro y Gestión de Zonas", padding="10")
        filtros_gestion_frame.pack(fill="x", pady=5)
        ttk.Label(filtros_gestion_frame, text="Centro Dist.:").pack(fill='x', pady=2)
        self.combo_cd = ttk.Combobox(filtros_gestion_frame, state="readonly")
        self.combo_cd.bind("<<ComboboxSelected>>", self.cargar_zonas_por_cd)
        self.combo_cd.pack(fill='x', padx=5)
        ttk.Label(filtros_gestion_frame, text="Zona Geográfica:").pack(fill='x', pady=2)
        self.combo_zona = ttk.Combobox(filtros_gestion_frame, state="readonly")
        self.combo_zona.bind("<<ComboboxSelected>>", self.cargar_clientes_por_zona)
        self.combo_zona.pack(fill='x', padx=5)
        ttk.Separator(filtros_gestion_frame, orient='horizontal').pack(fill='x', pady=5)
        ttk.Label(filtros_gestion_frame, text="Gestión de Estructura:").pack(fill='x', pady=2)
        ttk.Button(filtros_gestion_frame, text="Agregar Nuevo CD", command=self.agregar_cd).pack(fill='x', pady=2)
        ttk.Button(filtros_gestion_frame, text="Agregar Nueva Zona al CD", command=self.agregar_zona_al_cd).pack(fill='x', pady=2)
        ttk.Button(filtros_gestion_frame, text="Editar Zona Geográfica", command=self.editar_zona).pack(fill='x', pady=2)
        ttk.Button(filtros_gestion_frame, text="Eliminar Zona y Clientes", command=self.eliminar_zona).pack(fill='x', pady=2)
        rutas_rec_frame = ttk.LabelFrame(gestion_frame, text="Rutas Recurrentes", padding="10")
        rutas_rec_frame.pack(fill="x", pady=5)
        ttk.Button(rutas_rec_frame, text="Guardar Ruta Actual", command=self.guardar_ruta_recurrente).pack(fill='x', pady=2)
        ttk.Button(rutas_rec_frame, text="Cargar Ruta Guardada", command=self.cargar_ruta_recurrente).pack(fill='x')
        clientes_frame = ttk.LabelFrame(main_frame, text="3. Clientes con Despacho para Hoy", padding="10")
        clientes_frame.grid(row=1, column=0, sticky="nsew", padx=5, pady=5)
        clientes_frame.grid_rowconfigure(0, weight=1)
        clientes_frame.grid_columnconfigure(0, weight=1)
        listbox_frame = ttk.Frame(clientes_frame)
        listbox_frame.grid(row=0, column=0, sticky="nsew")
        listbox_frame.grid_rowconfigure(0, weight=1)
        listbox_frame.grid_columnconfigure(0, weight=1)
        scrollbar = ttk.Scrollbar(listbox_frame, orient=tk.VERTICAL)
        self.listbox_clientes = tk.Listbox(listbox_frame, selectmode=tk.MULTIPLE, yscrollcommand=scrollbar.set, background="#fdfdfd", foreground="#1e1e1e", borderwidth=0, highlightthickness=0)
        scrollbar.config(command=self.listbox_clientes.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.listbox_clientes.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        salida_frame = ttk.LabelFrame(main_frame, text="4. Resultados de la Ruta", padding="10")
        salida_frame.grid(row=2, column=0, sticky="nsew", padx=5, pady=5)
        salida_frame.grid_rowconfigure(0, weight=1)
        salida_frame.grid_columnconfigure(0, weight=1)
        self.salida_texto = tk.Text(salida_frame, height=10, width=60, state="disabled", wrap="word", background="#fdfdfd", foreground="#1e1e1e", borderwidth=0, highlightthickness=0)
        self.salida_texto.grid(row=0, column=0, sticky="nsew")
        self.style.configure("Accent.TButton", font=("Segoe UI", 12, "bold"))
        self.btn_calcular = ttk.Button(main_frame, text="Calcular Ruta Óptima", command=self.iniciar_calculo, style="Accent.TButton")
        self.btn_calcular.grid(row=3, column=0, columnspan=2, pady=10)

    def cargar_y_filtrar_despachos(self):
        self.despachos_pendientes, mensaje = procesar_despachos_del_dia()
        
        if self.despachos_pendientes is None:
            messagebox.showerror("Error", mensaje)
            return
        
        messagebox.showinfo("Proceso finalizado", mensaje)

        dias_semana = ["Lunes", "Martes", "Miércoles", "Jueves", "Viernes", "Sábado", "Domingo"]
        hoy = dias_semana[datetime.now().weekday()]
        
        self.listbox_clientes.delete(0, tk.END)

        clientes_db = db_query("SELECT id, nombre, destination_id, dias_reparto FROM clientes WHERE destination_id IS NOT NULL")
        
        clientes_para_hoy = []
        for cliente in clientes_db:
            cliente_id, nombre, dest_id, dias_reparto = cliente
            if dest_id in self.despachos_pendientes:
                if not dias_reparto or hoy in dias_reparto:
                    total_cajas = self.despachos_pendientes[dest_id]
                    clientes_para_hoy.append((cliente_id, f"{nombre} - {int(total_cajas)} cajas"))

        clientes_para_hoy.sort(key=lambda x: x[1])
        for cliente_id, texto in clientes_para_hoy:
            self.listbox_clientes.insert(tk.END, f"{cliente_id} | {texto}")
        
        self.salida_texto.config(state="normal")
        self.salida_texto.delete("1.0", tk.END)
        self.salida_texto.insert(tk.END, f"Mostrando {len(clientes_para_hoy)} clientes con despacho para hoy ({hoy}).")
        self.salida_texto.config(state="disabled")
    
    def mostrar_historial_rutas(self):
        top = tk.Toplevel(self.root)
        top.title("Historial de Rutas")
        top.geometry("600x400")

        tree_frame = ttk.Frame(top, padding="10")
        tree_frame.pack(fill=tk.BOTH, expand=True)

        tree = ttk.Treeview(tree_frame, columns=("Fecha", "Patente", "Clientes"), show="headings")
        tree.heading("Fecha", text="Fecha"); tree.column("Fecha", width=150)
        tree.heading("Patente", text="Patente"); tree.column("Patente", width=100, anchor="center")
        tree.heading("Clientes", text="Clientes"); tree.column("Clientes", width=250)
        tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        scrollbar = ttk.Scrollbar(tree_frame, orient="vertical", command=tree.yview)
        scrollbar.pack(side=tk.RIGHT, fill="y")
        tree.configure(yscrollcommand=scrollbar.set)
        
        def refrescar_historial():
            tree.delete(*tree.get_children())
            query = """
                SELECT r.id, r.fecha, c.patente, r.clientes
                FROM rutas r
                JOIN camiones c ON r.camion_id = c.id
                ORDER BY r.fecha DESC
            """
            for row in db_query(query):
                clientes_ids_str = row[3]
                if clientes_ids_str:
                    clientes_ids = [int(cid) for cid in clientes_ids_str.split(',')]
                    nombres_clientes = [c[1] for c in self.all_clientes_data if c[0] in clientes_ids]
                    clientes_mostrar = ", ".join(nombres_clientes)
                else:
                    clientes_mostrar = "(sin clientes)"
                tree.insert("", tk.END, values=(row[1], row[2], clientes_mostrar), tags=(clientes_ids_str,))

        def ver_detalle():
            if not tree.selection():
                messagebox.showwarning("Atención", "Selecciona una ruta del historial.", parent=top)
                return
            item = tree.item(tree.selection()[0])
            clientes_str = item['tags'][0]
            if not clientes_str:
                messagebox.showinfo("Info", "Esta ruta no tiene clientes registrados.", parent=top)
                return
            btn_ver_mapa.config(state="disabled", text="Generando...")
            clientes_ids = [int(cid) for cid in clientes_str.split(',')]
            thread = threading.Thread(target=self.generar_mapa_historial, args=(clientes_ids, btn_ver_mapa))
            thread.daemon = True
            thread.start()

        botones_frame = ttk.Frame(top, padding="10")
        botones_frame.pack(fill=tk.X)
        btn_ver_mapa = ttk.Button(botones_frame, text="Ver Ruta en Mapa", command=ver_detalle, style="Accent.TButton")
        btn_ver_mapa.pack(side=tk.RIGHT, padx=5)
        ttk.Button(botones_frame, text="Refrescar", command=refrescar_historial).pack(side=tk.RIGHT)
        refrescar_historial()

    def generar_mapa_historial(self, clientes_ids, boton):
        try:
            coords, nombres = [], []
            cliente_map = {c[0]: c for c in self.all_clientes_data}

            for cid in clientes_ids:
                cliente = cliente_map.get(cid)
                if cliente:
                    coords.append([cliente[2], cliente[3]]) # lon, lat
                    nombres.append(cliente[1])
            
            if not coords:
                raise ValueError("No se encontraron las coordenadas para los clientes de esta ruta.")

            coords.insert(0, [-72.093775, -36.562653]) # Bodega
            nombres.insert(0, "Bodega (Inicio)")
            route_geojson = client.directions(coordinates=coords, profile='driving-hgv', format='geojson')
            m = folium.Map(location=coords[1][::-1], zoom_start=10) # Centrar en el primer cliente
            folium.GeoJson(route_geojson, style_function=lambda x: {'color':'purple','weight':4}).add_to(m)
            
            for i, (coord, nombre) in enumerate(zip(coords, nombres)):
                folium.Marker([coord[1], coord[0]], popup=f"{i}. {nombre}",
                              icon=folium.Icon(color="green" if i == 0 else "purple")).add_to(m)

            map_file = f"ruta_{datetime.now().strftime('%Y%m%d_%H%M%S')}.html"
            m.save(map_file)
            def finalizar():
                webbrowser.open(map_file)
                boton.config(state="normal", text="Ver Ruta en Mapa")
            self.root.after(0, finalizar)
        except Exception as e:
            def finalizar_con_error():
                messagebox.showerror("Error", f"No se pudo generar el mapa: {e}")
                boton.config(state="normal", text="Ver Ruta en Mapa")
            self.root.after(0, finalizar_con_error)

    def refrescar_datos_locales_y_ui(self):
        self.all_clientes_data = db_query("""
            SELECT 
                c.id, c.nombre, c.lon, c.lat, c.tipo_camion, z.cd, z.nombre 
            FROM 
                clientes c 
            JOIN 
                zonas z ON c.zona_id = z.id
        """)
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

    def agregar_cd(self):
        nuevo_cd_code = simpledialog.askstring("Agregar CD", "Ingresa el CÓDIGO de la nueva Sede/CD (ej. 6010):")
        if not nuevo_cd_code or not nuevo_cd_code.isdigit():
            if nuevo_cd_code: messagebox.showerror("Error", "El código CD debe ser numérico.", parent=self.root)
            return
        zona_inicial = simpledialog.askstring("Zona Inicial", f"Ingresa una ZONA GEOGRÁFICA inicial para el CD '{nuevo_cd_code}' (ej. Norte):")
        if not zona_inicial:
            messagebox.showwarning("Atención", "No se puede crear un CD sin al menos una zona inicial.")
            return

        try:
            db_execute("INSERT INTO zonas (nombre, cd) VALUES (?, ?)", (zona_inicial, nuevo_cd_code))
            messagebox.showinfo("Éxito", f"CD '{nuevo_cd_code}' y zona '{zona_inicial}' agregados.", parent=self.root)
            self.refrescar_datos_locales_y_ui() # Recarga los combobox
            self.combo_cd.set(nuevo_cd_code)
            self.cargar_zonas_por_cd()
            self.combo_zona.set(zona_inicial)
        except sqlite3.IntegrityError:
             messagebox.showerror("Error", "El CD o la Zona ya existe.", parent=self.root)
        except Exception as e:
            messagebox.showerror("Error", f"Ocurrió un error: {e}", parent=self.root)

    def agregar_zona_al_cd(self):
        cd = self.combo_cd.get()
        if not cd:
            messagebox.showwarning("Atención", "Selecciona un CD primero.")
            return
        nueva_zona = simpledialog.askstring("Agregar Zona", f"Nombre de la nueva Zona Geográfica para el CD '{cd}' (ej. Sur):")
        if nueva_zona:
            try:
                db_execute("INSERT INTO zonas (nombre, cd) VALUES (?, ?)", (nueva_zona, cd))
                messagebox.showinfo("Éxito", f"Zona '{nueva_zona}' agregada al CD '{cd}'.")
                self.refrescar_datos_locales_y_ui()
                self.combo_zona.set(nueva_zona)
            except sqlite3.IntegrityError:
                messagebox.showerror("Error", f"La zona '{nueva_zona}' ya existe en el CD '{cd}'.")
            except Exception as e:
                messagebox.showerror("Error", f"Ocurrió un error: {e}")
    
    def editar_zona(self):
        cd, zona_actual = self.combo_cd.get(), self.combo_zona.get()
        if not cd or not zona_actual:
            messagebox.showwarning("Atención", "Selecciona un CD y una Zona para editar.")
            return
        nuevo_nombre = simpledialog.askstring("Editar Zona", f"Nuevo nombre para la zona '{zona_actual}' (CD: {cd}):", initialvalue=zona_actual)
        if nuevo_nombre and nuevo_nombre != zona_actual:
            if messagebox.askyesno("Confirmar", f"¿Seguro de renombrar la zona '{zona_actual}' a '{nuevo_nombre}'? Esto afectará a TODOS los clientes asignados a esta zona."):
                try:
                    zona_id = db_query("SELECT id FROM zonas WHERE nombre=? AND cd=?", (zona_actual, cd))
                    if not zona_id:
                        messagebox.showerror("Error", "No se encontró la zona para editar.")
                        return
                    zona_id = zona_id[0][0]
                    db_execute("UPDATE zonas SET nombre=? WHERE id=?", (nuevo_nombre, zona_id))
                    
                    messagebox.showinfo("Éxito", "La zona ha sido renombrada.")
                    self.refrescar_datos_locales_y_ui()
                except Exception as e:
                    messagebox.showerror("Error", f"No se pudo editar la zona: {e}")

    def eliminar_zona(self):
        cd, zona = self.combo_cd.get(), self.combo_zona.get()
        if not cd or not zona:
            messagebox.showwarning("Atención", "Selecciona un CD y una Zona para eliminar.")
            return

        zona_id_data = db_query("SELECT id FROM zonas WHERE nombre=? AND cd=?", (zona, cd))
        if not zona_id_data:
             messagebox.showwarning("Atención", "La zona seleccionada no existe.")
             return
        zona_id = zona_id_data[0][0]

        if messagebox.askyesno("Confirmar Eliminación", f"¿SEGURO que deseas eliminar la zona '{zona}' del CD '{cd}' y **TODOS** sus clientes? Esta acción es irreversible."):
            try:
                db_execute("DELETE FROM clientes WHERE zona_id = ?", (zona_id,))
                
                db_execute("DELETE FROM zonas WHERE id = ?", (zona_id,))
                
                messagebox.showinfo("Éxito", f"La zona '{zona}' y sus clientes han sido eliminados.")
                self.refrescar_datos_locales_y_ui()
            except Exception as e:
                messagebox.showerror("Error", f"No se pudo completar la eliminación: {e}")

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
        cliente_id = int(self.listbox_clientes.get(seleccion[0]).split(" | ")[0])
        cliente = db_query("SELECT * FROM clientes WHERE id = ?", (cliente_id,))[0]
        cliente = next((c for c in self.all_clientes_data if c[0] == cliente_id), None)
        if not cliente: return
        top = tk.Toplevel(self.root); top.title(f"Editar Cliente: {cliente[1]}"); top.geometry("300x300")
        ttk.Label(top, text="Nombre:").pack(pady=5); entry_nombre = ttk.Entry(top, width=30); entry_nombre.insert(0, cliente[1]); entry_nombre.pack()
        ttk.Label(top, text="Longitud (Lon):").pack(pady=5); entry_lon = ttk.Entry(top, width=30); entry_lon.insert(0, cliente[2]); entry_lon.pack()
        ttk.Label(top, text="Latitud (Lat):").pack(pady=5); entry_lat = ttk.Entry(top, width=30); entry_lat.insert(0, cliente[3]); entry_lat.pack()
        ttk.Label(top, text="Tipo de camión req.:").pack(pady=5); combo_tipo = ttk.Combobox(top, values=["Largo", "Corto", "Dolly"], state="readonly"); combo_tipo.set(cliente[4]); combo_tipo.pack()
        ttk.Label(top, text="ID de Destino (del Excel):").pack(pady=5); entry_dest_id = ttk.Entry(top, width=30); 
        entry_dest_id.insert(0, cliente[6] if len(cliente)>6 else "") # Columna 6: destination_id
        entry_dest_id.pack()
        ttk.Label(top, text="Días de Reparto (ej: Lunes,Viernes):").pack(pady=5)
        entry_dias = ttk.Entry(top, width=30)
        entry_dias.insert(0, cliente[7] if len(cliente)>7 else "") # Columna 7: dias_reparto
        entry_dias.pack()

        def guardar():
            nombre, lon_str, lat_str, tipo = entry_nombre.get().strip(), entry_lon.get().strip(), entry_lat.get().strip(), combo_tipo.get()
            dest_id_str = entry_dest_id.get().strip()
            dias_reparto = entry_dias.get().strip()
            if not all([nombre, lon_str, lat_str]):
                messagebox.showwarning("Campos Vacíos", "Todos los campos son obligatorios.", parent=top); return
            if messagebox.askyesno("Confirmar", "¿Guardar los cambios en este cliente?", parent=top):
                try:
                    dest_id = int(dest_id_str)
                    lon, lat = float(lon_str), float(lat_str)
                    db_execute("UPDATE clientes SET nombre=?, lon=?, lat=?, tipo_camion=?, destination_id=?, dias_reparto=? WHERE id=?", (nombre, lon, lat, tipo, dest_id, dias_reparto, cliente_id))
                    messagebox.showinfo("Éxito", "Cliente actualizado.", parent=top)
                    top.destroy()
                    self.refrescar_datos_locales_y_ui()
                except ValueError:
                    messagebox.showerror("Error", "El ID de Destino debe ser un número.")
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
        clientes_ids = [int(self.listbox_clientes.get(i).split(" | ")[0]) for i in seleccion]
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

    def guardar_ruta_recurrente(self):
        seleccion = self.listbox_clientes.curselection()
        if len(seleccion) < 2:
            messagebox.showwarning("Atención", "Selecciona al menos 2 clientes para guardar una ruta.", parent=self.root)
            return

        nombre_ruta = simpledialog.askstring("Guardar Ruta", "Ingresa un nombre para esta ruta recurrente:", parent=self.root)
        if not nombre_ruta or not nombre_ruta.strip():
            return

        try:
            clientes_ids = [self.listbox_clientes.get(i).split(" | ")[0] for i in seleccion]
            clientes_ids_str = ",".join(clientes_ids)
            existe = db_query("SELECT id FROM rutas_recurrentes WHERE nombre_ruta = ?", (nombre_ruta,))
            if existe:
                if messagebox.askyesno("Confirmar", f"La ruta '{nombre_ruta}' ya existe. ¿Deseas sobreescribirla?"):
                    db_execute("UPDATE rutas_recurrentes SET clientes_ids = ? WHERE nombre_ruta = ?", (clientes_ids_str, nombre_ruta))
                else:
                    return
            else:
                db_execute("INSERT INTO rutas_recurrentes (nombre_ruta, clientes_ids) VALUES (?, ?)", (nombre_ruta, clientes_ids_str))
            
            messagebox.showinfo("Éxito", f"Ruta '{nombre_ruta}' guardada correctamente.")
        except Exception as e:
            messagebox.showerror("Error", f"No se pudo guardar la ruta: {e}")

    def cargar_ruta_recurrente(self):
        rutas = db_query("SELECT nombre_ruta FROM rutas_recurrentes ORDER BY nombre_ruta")
        if not rutas:
            messagebox.showinfo("Información", "No hay rutas recurrentes guardadas.", parent=self.root)
            return
        top = tk.Toplevel(self.root)
        top.title("Cargar Ruta Recurrente")
        top.geometry("300x400")
        ttk.Label(top, text="Selecciona una ruta para cargar:").pack(pady=10)
        listbox_rutas = tk.Listbox(top)
        listbox_rutas.pack(fill="both", expand=True, padx=10, pady=5)
        for ruta in rutas:
            listbox_rutas.insert(tk.END, ruta[0])
            
        def on_select():
            if not listbox_rutas.curselection():
                return
            
            nombre_ruta_sel = listbox_rutas.get(listbox_rutas.curselection()[0])
            clientes_ids_str = db_query("SELECT clientes_ids FROM rutas_recurrentes WHERE nombre_ruta = ?", (nombre_ruta_sel,))[0][0]
            clientes_ids_a_seleccionar = set(map(int, clientes_ids_str.split(',')))
            
            self.listbox_clientes.selection_clear(0, tk.END)
            
            for i in range(self.listbox_clientes.size()):
                item_id = int(self.listbox_clientes.get(i).split(" | ")[0])
                if item_id in clientes_ids_a_seleccionar:
                    self.listbox_clientes.selection_set(i)
            
            top.destroy()

        ttk.Button(top, text="Cargar Selección", command=on_select, style="Accent.TButton").pack(pady=10)

class Login:
    def __init__(self, root):
        self.root = root
        self.root.title("Login - Ruteador de Camiones")
        self.root.geometry("300x200")
        ttk.Label(root, text="Usuario:").pack(pady=5)
        self.entry_user = ttk.Entry(root); self.entry_user.pack()
        ttk.Label(root, text="Contraseña:").pack(pady=5)
        self.entry_pass = ttk.Entry(root, show="*"); self.entry_pass.pack()
        ttk.Button(root, text="Ingresar", command=self.verificar_login).pack(pady=20)
        self.rol_usuario = None

    def verificar_login(self):
        usuario = self.entry_user.get().strip()
        password = self.entry_pass.get().strip()
        resultado = db_query("SELECT rol FROM usuarios WHERE usuario=? AND password=?", (usuario, password))
        if resultado:
            self.rol_usuario = resultado[0][0]
            self.root.destroy()
        else:
            messagebox.showerror("Error", "Usuario o contraseña incorrectos.")

if __name__ == "__main__":
    if not client:
        print("Error: No se puede iniciar la aplicación sin una clave de API válida en config.ini")
    else:
        login_root = tk.Tk()
        sv_ttk.set_theme("light")
        login = Login(login_root)
        login_root.mainloop()
        if login.rol_usuario:
            root = tk.Tk()
            sv_ttk.set_theme("light")
            app = App(root, login.rol_usuario)
            root.mainloop()