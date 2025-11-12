import tkinter as tk
from tkinter import messagebox, ttk, simpledialog
import threading
import re
import webbrowser
from datetime import datetime, timedelta
import sqlite3
import math

from db import  (
    get_all_clientes_with_details, get_distinct_cds_from_clientes, get_zonas_by_cd_from_clientes,
    add_zona, get_zona_id, add_cliente, update_cliente, delete_cliente,
    get_all_camiones, get_historial_rutas, get_camion_id_by_patente, add_camion,importar_clientes_excel
)
from excel_parser import procesar_despachos_del_dia, procesar_fix_planning
from route_creator import proceso_de_calculo, client


def validar_patente(patente):
    return bool(re.fullmatch(r"[A-Z]{4}\d{2}|[A-Z]{2}\d{4}", patente.upper()))

class App:
    def __init__(self, root, rol):
        self.rol = rol
        self.root = root
        self.root.title("Ruteador de Camiones v0.4")
        self.root.geometry("850x700")
        self.style = ttk.Style()
        self.style.configure(".", font=("Segoe UI", 10))
        self.clientes_ya_ruteados = set()
        self.target_cajas = tk.StringVar(value="10000") #valor de cajas x trailer, cambiar después para que cada CD tenga el suyo
        self.all_clientes_data = []
        self.despachos_pendientes = {}
        

        self.setup_ui()
        self.refrescar_datos_locales_y_ui()

    BODEGAS_CENTRALES = {
        "6003": [-70.30957240719687, -23.76044036099112],
        "6010": [-72.093775, -36.562653],
        "6004": [-72.62655718628656, -38.7713653407215],
    }

    def setup_ui(self):
        self.style = ttk.Style()
        COLOR_ROJO = "#D40511"
        COLOR_AMARILLO = "#FFCC00"
        COLOR_TEXTO_BLANCO = "#FFFFFF"

        self.style.configure("Accent.TButton",
                            background=COLOR_ROJO,
                            foreground=COLOR_TEXTO_BLANCO,
                            font=("Segoe UI", 12, "bold"),
                            padding=(10, 5))
        self.style.map("Accent.TButton",
                    background=[('active', '#A8040E'), ('pressed', '#A8040E')],
                    foreground=[('active', COLOR_TEXTO_BLANCO)])
        self.style.configure("TLabelFrame.Label",
                            foreground=COLOR_ROJO,
                            font=("Segoe UI", 11, "bold"))
        self.style.configure("TNotebook.Tab",
                            font=("Segoe UI", 10, "bold"),
                            padding=(10, 5))
        self.style.map("TNotebook.Tab",
                    background=[("selected", COLOR_ROJO)],
                    foreground=[("selected", COLOR_TEXTO_BLANCO)])
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky="nsew")
        self.root.grid_columnconfigure(0, weight=1)
        self.root.grid_rowconfigure(0, weight=1)
        main_frame.grid_columnconfigure(0, weight=1, minsize=400)
        main_frame.grid_columnconfigure(1, weight=0)
        main_frame.grid_rowconfigure(1, weight=1)

        top_frame = ttk.Frame(main_frame)
        top_frame.grid(row=0, column=0, sticky="ew", padx=5, pady=5)
        top_frame.grid_columnconfigure(1, weight=0) # <-- El nuevo frame no se expande
        top_frame.grid_columnconfigure(2, weight=1) # <-- El frame de camión (ahora col 2) sí

        self.btn_cargar_excel = ttk.Button(top_frame, text="1. Cargar Despachos (Excel)", command=self.cargar_y_filtrar_despachos, style="Accent.TButton")
        self.btn_cargar_excel.grid(row=0, column=0, padx=(0, 10), pady=5, sticky="ew")

        sugerir_frame = ttk.LabelFrame(top_frame, text="1.5 Sugerir Carga", padding="10")
        sugerir_frame.grid(row=0, column=1, padx=5, pady=5, sticky="ew")
        
        ttk.Label(sugerir_frame, text="Target Cajas x Trailer:").pack(side=tk.LEFT, padx=5)
        
        entry_target = ttk.Entry(sugerir_frame, width=10, textvariable=self.target_cajas)
        entry_target.pack(side=tk.LEFT, padx=5)
        
        btn_sugerir = ttk.Button(sugerir_frame, text="Sugerir", command=self.sugerir_carga)
        btn_sugerir.pack(side=tk.LEFT, padx=5)

        camion_frame = ttk.LabelFrame(top_frame, text="2. Datos del Camión", padding="10")
        camion_frame.grid(row=0, column=2, padx=5, pady=5, sticky="ew") 
        ttk.Label(camion_frame, text="Patente:").pack(side=tk.LEFT, padx=5)
        self.entry_patente = ttk.Entry(camion_frame, width=15)
        self.entry_patente.pack(side=tk.LEFT, padx=5)
        ttk.Label(camion_frame, text="Tipo:").pack(side=tk.LEFT, padx=5)
        self.combo_tipo = ttk.Combobox(camion_frame, values=["Largo", "Corto", "Dolly"], state="readonly", width=10)
        self.combo_tipo.current(0)
        self.combo_tipo.pack(side=tk.LEFT, padx=5)

        panel_derecho = ttk.Notebook(main_frame)
        panel_derecho.grid(row=0, column=1, rowspan=3, sticky="ns", padx=5, pady=5)

        tab_filtros = ttk.Frame(panel_derecho, padding="10")
        tab_gestion = ttk.Frame(panel_derecho, padding="10")
        tab_herramientas = ttk.Frame(panel_derecho, padding="10")

        panel_derecho.add(tab_filtros, text="Filtros")
        panel_derecho.add(tab_gestion, text="Gestión")
        panel_derecho.add(tab_herramientas, text="Herramientas")

        ttk.Label(tab_filtros, text="Centro Dist.:").pack(fill='x', pady=2)
        self.combo_cd = ttk.Combobox(tab_filtros, state="readonly")
        self.combo_cd.bind("<<ComboboxSelected>>", self.cargar_zonas_por_cd)
        self.combo_cd.pack(fill='x', padx=5, pady=(0, 10))
        ttk.Label(tab_filtros, text="Zona Geográfica:").pack(fill='x', pady=2)
        self.combo_zona = ttk.Combobox(tab_filtros, state="readonly")
        self.combo_zona.bind("<<ComboboxSelected>>", self.refrescar_lista_clientes)
        self.combo_zona.pack(fill='x', padx=5)

        self.sobrantes_var = tk.BooleanVar()
        chk_sobrantes = ttk.Checkbutton(tab_filtros, 
                                        text="Mostrar solo sobrantes (de todas las zonas)", 
                                        variable=self.sobrantes_var, 
                                        command=self.on_check_sobrantes_changed)
        chk_sobrantes.pack(fill='x', padx=5, pady=(10, 0))

        clientes_gestion_frame = ttk.LabelFrame(tab_gestion, text="Clientes", padding="10")
        clientes_gestion_frame.pack(fill="x", pady=5)
        ttk.Button(clientes_gestion_frame, text="Importar Clientes (Excel)", command=self.llamar_importar_clientes).pack(fill='x', pady=2)
        #ttk.Button(clientes_gestion_frame, text="Agregar Cliente", command=self.agregar_cliente).pack(fill='x', pady=2)
        ttk.Button(clientes_gestion_frame, text="Editar Cliente", command=self.editar_cliente_desde_lista).pack(fill='x')
        ttk.Button(clientes_gestion_frame, text="Ver Todos los Clientes", command=self.mostrar_todos_los_clientes).pack(fill='x')
        #ttk.Button(clientes_gestion_frame, text="Eliminar Cliente", command=self.eliminar_cliente).pack(fill='x', pady=2)

        zonas_cd_gestion_frame = ttk.LabelFrame(tab_gestion, text="Zonas y CDs", padding="10")
        zonas_cd_gestion_frame.pack(fill="x", pady=5)
        #ttk.Button(zonas_cd_gestion_frame, text="Agregar Nuevo CD", command=self.agregar_cd).pack(fill='x', pady=2)
        #ttk.Button(zonas_cd_gestion_frame, text="Agregar Zona a CD", command=self.agregar_zona_al_cd).pack(fill='x', pady=2)

        camiones_gestion_frame = ttk.LabelFrame(tab_gestion, text="Camiones", padding="10")
        camiones_gestion_frame.pack(fill="x", pady=5)
        ttk.Button(camiones_gestion_frame, text="Ver/Editar Camiones", command=self.mostrar_camiones).pack(fill="x", pady=2)

        ttk.Button(tab_herramientas, text="Historial de Rutas", command=self.mostrar_historial_rutas).pack(fill="x", pady=5)
        ttk.Button(tab_herramientas, text="Actualizar Días (FixPlanning)", command=self.actualizar_dias_reparto).pack(fill="x", pady=5)

        clientes_frame = ttk.LabelFrame(main_frame, text="3. Clientes a Rutear", padding="10")
        clientes_frame.grid(row=1, column=0, sticky="nsew", padx=5, pady=5)
        clientes_frame.grid_rowconfigure(0, weight=1)
        clientes_frame.grid_columnconfigure(0, weight=1)
        
        scrollbar = ttk.Scrollbar(clientes_frame, orient=tk.VERTICAL)
        self.listbox_clientes = tk.Listbox(clientes_frame, selectmode=tk.MULTIPLE, yscrollcommand=scrollbar.set, borderwidth=0, highlightthickness=0, font=("Segoe UI", 11))
        scrollbar.config(command=self.listbox_clientes.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.listbox_clientes.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        salida_frame = ttk.LabelFrame(main_frame, text="4. Resultados de la Ruta", padding="10")
        salida_frame.grid(row=2, column=0, sticky="nsew", padx=5, pady=5)
        salida_frame.grid_rowconfigure(0, weight=1)
        salida_frame.grid_columnconfigure(0, weight=1)
        self.salida_texto = tk.Text(salida_frame, height=8, state="disabled", wrap="word", borderwidth=0, highlightthickness=0, font=("Consolas", 10))
        self.salida_texto.grid(row=0, column=0, sticky="nsew")

        self.btn_calcular = ttk.Button(main_frame, text="Calcular Ruta Óptima", command=self.iniciar_calculo, style="Accent.TButton")
        self.btn_calcular.grid(row=3, column=0, columnspan=2, pady=10, sticky="ew")  

    def actualizar_dias_reparto(self):
        if messagebox.askyesno("Confirmar Actualización",
                            "Esto sobrescribirá los días de reparto de los clientes según el archivo FixPlanning.\n¿Deseas continuar?"):
            mensaje = procesar_fix_planning()
            messagebox.showinfo("Resultado del Proceso", mensaje)
            self.refrescar_datos_locales_y_ui()

    def llamar_importar_clientes(self):
        mensaje = importar_clientes_excel()
        messagebox.showinfo("Resultado de Importación", mensaje, parent=self.root)
        if "Éxito" in mensaje:
            self.refrescar_datos_locales_y_ui()

    def refrescar_datos_locales_y_ui(self):
        self.all_clientes_data = get_all_clientes_with_details()
        self.cargar_cds()

    def refrescar_lista_clientes(self, event=None):
        cd_filtro = self.combo_cd.get()
        zona_filtro = self.combo_zona.get()
        self.listbox_clientes.delete(0, tk.END)

        mostrar_solo_sobrantes = self.sobrantes_var.get()

        # Obtener coordenadas de la bodega para ordenar
        can_sort_by_distance = False
        start_lat, start_lon = None, None
        
        # Obtenemos las coords de la bodega del CD seleccionado
        bodega_coords_lon_lat = self.BODEGAS_CENTRALES.get(cd_filtro) 
        
        if bodega_coords_lon_lat:
            start_lon, start_lat = bodega_coords_lon_lat[0], bodega_coords_lon_lat[1]
            can_sort_by_distance = True

        if self.despachos_pendientes:
            dias_semana = ["Lunes", "Martes", "Miercoles", "Jueves", "Viernes", "Sábado", "Domingo"]
            fecha_manana = datetime.now() + timedelta(days=1)
            dia_manana = dias_semana[fecha_manana.weekday()]
            
            clientes_para_mostrar = []
            clientes_fuera_filtro = 0 

            for cliente in self.all_clientes_data:
                # cliente[2] = lon, cliente[3] = lat
                cliente_id, nombre, cliente_lon, cliente_lat, _, cd_cliente, zona_cliente, dest_id, dias_reparto, _, _ = cliente
                
                if dest_id in self.despachos_pendientes:
                    total_cajas = self.despachos_pendientes[dest_id] 
                    tiene_despacho_valido = total_cajas > 0 and (not dias_reparto or dia_manana in dias_reparto)

                    if tiene_despacho_valido:
                        if cliente_id in self.clientes_ya_ruteados:
                            continue

                        display_name = nombre if nombre else f"Local #{dest_id}"
                        
                        # Calcular distancia
                        distancia = float('inf')
                        if can_sort_by_distance:
                            distancia = self._calculate_haversine(start_lat, start_lon, cliente_lat, cliente_lon)
                        
                        # Añadimos la distancia y el display_name (para orden alfabético si falla la distancia)
                        cliente_tuple = (cliente_id, f"{display_name} - {int(total_cajas)} cajas", display_name, distancia)
                        
                        if mostrar_solo_sobrantes:
                            clientes_para_mostrar.append(cliente_tuple)
                        else:
                            if cd_cliente == cd_filtro and zona_cliente == zona_filtro:
                                clientes_para_mostrar.append(cliente_tuple)
                            else:
                                clientes_fuera_filtro += 1
            
            # Lógica de ordenamiento
            if can_sort_by_distance and not mostrar_solo_sobrantes:
                # Ordenar por distancia (índice 3)
                clientes_para_mostrar.sort(key=lambda x: x[3])
            else:
                # Ordenar alfabéticamente (índice 2)
                clientes_para_mostrar.sort(key=lambda x: x[2])
            
            # Desempacar 4 elementos
            for cliente_id, texto, _, _ in clientes_para_mostrar:
                self.listbox_clientes.insert(tk.END, f"{cliente_id} | {texto}")
            
            # Actualizar el texto de feedback
            self.salida_texto.config(state="normal")
            self.salida_texto.delete("1.0", tk.END)
            
            if mostrar_solo_sobrantes:
                 mensaje_feedback = f"Mostrando {len(clientes_para_mostrar)} locales sobrantes (pendientes de ruteo) de TODAS las zonas."
            else:
                mensaje_feedback = f"Mostrando {len(clientes_para_mostrar)} locales con despacho para mañana ({dia_manana}) en {cd_filtro} - {zona_filtro}."
                # Añadir feedback de ordenamiento
                if can_sort_by_distance:
                    mensaje_feedback += "\n(Ordenados por cercanía a la bodega)."
                else:
                    mensaje_feedback += "\n(Ordenados alfabéticamente)."

                if clientes_fuera_filtro > 0:
                    mensaje_feedback += f"\nSe omitieron {clientes_fuera_filtro} locales con despacho de otras zonas/CDs."
            
            self.salida_texto.insert(tk.END, mensaje_feedback)
            self.salida_texto.config(state="disabled")

        else:
            # Lógica para cuando NO hay despachos cargados
            self.salida_texto.config(state="normal")
            self.salida_texto.delete("1.0", tk.END)
            
            if mostrar_solo_sobrantes:
                self.salida_texto.insert(tk.END, "Modo 'Sobrantes': Carga primero un archivo de Despachos.")
            else:
                # Lógica original (mostrar clientes maestros)
                clientes_filtrados = [c for c in self.all_clientes_data if c[5] == cd_filtro and c[6] == zona_filtro]
                clientes_filtrados.sort(key=lambda x: x[1] or f"Local {x[7]}")

                for cliente in clientes_filtrados:
                    display_name = cliente[1] if cliente[1] else f"Local #{cliente[7]}"
                    self.listbox_clientes.insert(tk.END, f"{cliente[0]} | {display_name}")
                
                self.salida_texto.insert(tk.END, f"Mostrando {len(clientes_filtrados)} locales maestros en {cd_filtro} - {zona_filtro}.")
            
            self.salida_texto.config(state="disabled")

    def _calculate_haversine(self, lat1, lon1, lat2, lon2):
        R = 6371
        try:
            if any(v is None for v in [lat1, lon1, lat2, lon2]):
                return float('inf')

            lat1_rad = math.radians(float(lat1))
            lon1_rad = math.radians(float(lon1))
            lat2_rad = math.radians(float(lat2))
            lon2_rad = math.radians(float(lon2))

            dlon = lon2_rad - lon1_rad
            dlat = lat2_rad - lat1_rad
            
            a = math.sin(dlat / 2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2)**2
            c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
            
            distance = R * c
            return distance
        except (ValueError, TypeError):
            return float('inf')

    def cargar_cds(self):
        cds_query = get_distinct_cds_from_clientes()
        
        cds = [row[0] for row in cds_query]
        self.combo_cd['values'] = cds
        if cds:
            self.combo_cd.set(cds[0])
        else:
            self.combo_cd.set("")
        self.cargar_zonas_por_cd()

    def cargar_zonas_por_cd(self, event=None):
        cd_seleccionado = self.combo_cd.get()

        zonas_query = get_zonas_by_cd_from_clientes(cd_seleccionado)
        
        zonas = [row[0] for row in zonas_query]
        self.combo_zona['values'] = zonas
        if zonas:
            self.combo_zona.set(zonas[0])
        else:
            self.combo_zona.set("")
        #self.cargar_clientes_por_zona()

    def cargar_clientes_por_zona(self, event=None):
        cd = self.combo_cd.get()
        zona = self.combo_zona.get()
        self.listbox_clientes.delete(0, tk.END)
        
        clientes_filtrados = [c for c in self.all_clientes_data if c[5] == cd and c[6] == zona]
        clientes_filtrados.sort(key=lambda x: x[1] or f"Local {x[7]}")

        for cliente in clientes_filtrados:
            display_name = cliente[1] if cliente[1] else f"Local #{cliente[7]}"
            self.listbox_clientes.insert(tk.END, f"{cliente[0]} | {display_name}")

    def cargar_y_filtrar_despachos(self):
        self.despachos_pendientes, mensaje = procesar_despachos_del_dia()
        
        if self.despachos_pendientes is None:
            messagebox.showerror("Error", mensaje)
            return
        
        messagebox.showinfo("Proceso finalizado", mensaje)

        cd_filtro = self.combo_cd.get()
        zona_filtro = self.combo_zona.get()
        
        dias_semana = ["Lunes", "Martes", "Miercoles", "Jueves", "Viernes", "Sábado", "Domingo"]
        fecha_manana = datetime.now() + timedelta(days=1)
        dia_manana = dias_semana[fecha_manana.weekday()]
        
        self.listbox_clientes.delete(0, tk.END)
        
        clientes_para_manana = []
        clientes_fuera_filtro = 0

        for cliente in self.all_clientes_data:
            cliente_id, nombre, _, _, _, cd_cliente, zona_cliente, dest_id, dias_reparto, _, _ = cliente
            
            if dest_id in self.despachos_pendientes:
                total_cajas = self.despachos_pendientes[dest_id] 
                tiene_despacho_valido = total_cajas > 0 and (not dias_reparto or dia_manana in dias_reparto)

                if tiene_despacho_valido:
                    if cd_cliente == cd_filtro and zona_cliente == zona_filtro:
                        display_name = nombre if nombre else f"Local #{dest_id}"
                        clientes_para_manana.append((cliente_id, f"{display_name} - {int(total_cajas)} cajas", display_name))
                    else:
                        clientes_fuera_filtro += 1

        clientes_para_manana.sort(key=lambda x: x[2])
        for cliente_id, texto, _ in clientes_para_manana:
            self.listbox_clientes.insert(tk.END, f"{cliente_id} | {texto}")
        
        self.salida_texto.config(state="normal")
        self.salida_texto.delete("1.0", tk.END)
        self.salida_texto.insert(tk.END, f"Mostrando {len(clientes_para_manana)} locales con despacho para mañana ({dia_manana}).")
        self.salida_texto.config(state="disabled")

    def agregar_cd(self):
        nuevo_cd = simpledialog.askstring("Agregar CD", "Código del nuevo CD (ej: 6010):", parent=self.root)
        if nuevo_cd:
            zona_inicial = simpledialog.askstring("Agregar Zona", f"Nombre de la zona inicial para el CD {nuevo_cd} (ej: Urbana):", parent=self.root)
            if zona_inicial:
                try:
                    add_zona(zona_inicial, nuevo_cd)
                    messagebox.showinfo("Éxito", "CD y Zona agregados.", parent=self.root)
                    self.refrescar_datos_locales_y_ui()
                except sqlite3.IntegrityError:
                    messagebox.showerror("Error", "Esa combinación de Zona y CD ya existe.", parent=self.root)

    def agregar_zona_al_cd(self):
        cd_seleccionado = self.combo_cd.get()
        if not cd_seleccionado:
            messagebox.showwarning("Atención", "Selecciona un CD primero.", parent=self.root)
            return
        
        nueva_zona = simpledialog.askstring("Agregar Zona", f"Nombre de la nueva zona para el CD {cd_seleccionado}:", parent=self.root)
        if nueva_zona:
            try:
                add_zona(nueva_zona, cd_seleccionado)
                self.refrescar_datos_locales_y_ui()
                self.combo_zona.set(nueva_zona)
            except sqlite3.IntegrityError:
                messagebox.showerror("Error", "Esa combinación de Zona y CD ya existe.", parent=self.root)
    
    def agregar_cliente(self):
        cd, zona = self.combo_cd.get(), self.combo_zona.get()
        if not cd or not zona:
            messagebox.showwarning("Atención", "Debes seleccionar un CD y una Zona.", parent=self.root); return
        
        # zona_id ya no es necesario, la tabla clientes guarda el texto de cd y zona.
        # zona_id = get_zona_id(zona, cd)
        # if not zona_id:
        #    messagebox.showerror("Error", "La zona seleccionada no se encontró en la base de datos.", parent=self.root); return

        top = tk.Toplevel(self.root); top.title("Agregar Cliente"); top.geometry("350x450")
        
        fields = ["ID Local (Excel):", "Nombre (Opcional):", "Longitud (Lon):", "Latitud (Lat):", "Días Reparto (,)"]
        entries = {}
        for field in fields:
            frm = ttk.Frame(top); frm.pack(fill='x', padx=10, pady=5)
            lbl = ttk.Label(frm, text=field, width=20); lbl.pack(side='left')
            entry = ttk.Entry(frm); entry.pack(side='right', expand=True, fill='x')
            entries[field] = entry

        frm_tipo = ttk.Frame(top); frm_tipo.pack(fill='x', padx=10, pady=5)
        lbl_tipo = ttk.Label(frm_tipo, text="Tipo Camión Req.:", width=20); lbl_tipo.pack(side='left')
        combo_tipo = ttk.Combobox(frm_tipo, values=["Largo", "Corto", "Dolly"], state="readonly"); combo_tipo.current(0); combo_tipo.pack(side='right', expand=True, fill='x')

        def guardar():
            try:
                dest_id = int(entries["ID Local (Excel):"].get().strip())
                nombre = entries["Nombre (Opcional):"].get().strip()
                lon = float(entries["Longitud (Lon):"].get().strip())
                lat = float(entries["Latitud (Lat):"].get().strip())
                dias = entries["Días Reparto (,)"].get().strip()
                tipo = combo_tipo.get()

                # Pasamos 'cd' y 'zona' (texto) en lugar de 'zona_id'
                add_cliente(nombre, lon, lat, tipo, cd, zona, dest_id, dias)
                messagebox.showinfo("Éxito", "Cliente agregado.", parent=top)
                top.destroy()
                self.refrescar_datos_locales_y_ui()
            except ValueError:
                messagebox.showerror("Error de Formato", "ID Local, Longitud y Latitud deben ser números válidos.", parent=top)
            except sqlite3.IntegrityError:
                messagebox.showerror("Error de Duplicado", "Ya existe un cliente con ese ID de Local.", parent=top)

        ttk.Button(top, text="Guardar Cliente", command=guardar, style="Accent.TButton").pack(pady=20)
    
    def mostrar_todos_los_clientes(self):
        top = tk.Toplevel(self.root)
        top.title("Gestión de Todos los Clientes")
        top.geometry("900x600")

        search_frame = ttk.Frame(top, padding=(10, 10, 10, 0))
        search_frame.pack(fill='x')
        ttk.Label(search_frame, text="Buscar por ID Local:").pack(side=tk.LEFT, padx=(0, 5))
        search_entry = ttk.Entry(search_frame)
        search_entry.pack(side=tk.LEFT, fill='x', expand=True, padx=5)

        tree_frame = ttk.Frame(top, padding=(10, 10, 10, 10))
        tree_frame.pack(fill='both', expand=True)

        cols = ("db_id", "local_id", "nombre", "formato", "cd", "zona", "dias_entrega", "t_carrier")
        display_cols = ("local_id", "nombre", "formato", "cd", "zona", "dias_entrega", "t_carrier")
        headings = ("DB_ID", "ID Local", "Nombre", "Formato", "CD", "Zona", "Días Entrega", "T. Camión")
        col_widths = (0, 80, 200, 100, 80, 100, 150, 80) 
        
        tree = ttk.Treeview(tree_frame, columns=cols, show="headings", displaycolumns=display_cols)
        
        for i, col in enumerate(cols):
            tree.heading(col, text=headings[i], anchor='w')
            tree.column(col, width=col_widths[i], anchor='w', stretch=True)

        # Scrollbars
        scrollbar_y = ttk.Scrollbar(tree_frame, orient=tk.VERTICAL, command=tree.yview)
        scrollbar_x = ttk.Scrollbar(tree_frame, orient=tk.HORIZONTAL, command=tree.xview)
        tree.configure(yscrollcommand=scrollbar_y.set, xscrollcommand=scrollbar_x.set)
        
        scrollbar_y.pack(side=tk.RIGHT, fill=tk.Y)
        scrollbar_x.pack(side=tk.BOTTOM, fill=tk.X)
        tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        def poblar_tabla(filtro=""):
            tree.delete(*tree.get_children()) 
            
            clientes_para_mostrar = self.all_clientes_data
            if filtro:
                clientes_para_mostrar = [
                    c for c in self.all_clientes_data 
                    if str(c[7]).strip().startswith(filtro) 
                ]
            
            clientes_para_mostrar.sort(key=lambda x: x[7]) 

            for cliente in clientes_para_mostrar:
                valores = (
                    cliente[0], # db_id (oculto)
                    cliente[7], # destination_number
                    cliente[1] if cliente[1] else "S/N", 
                    cliente[9] if cliente[9] else "S/F", 
                    cliente[5] if cliente[5] else "S/CD", 
                    cliente[6] if cliente[6] else "S/Z", 
                    cliente[8] if cliente[8] else "-",
                    cliente[4] if cliente[4] else "S/T" # t_carrier
                )
                tree.insert("", "end", values=valores)

        def ejecutar_busqueda(event=None):
            filtro = search_entry.get().strip()
            poblar_tabla(filtro)

        search_button = ttk.Button(search_frame, text="Buscar", command=ejecutar_busqueda)
        search_button.pack(side=tk.LEFT, padx=5)
        search_entry.bind("<Return>", ejecutar_busqueda)
        
        def editar_seleccion_tabla():
            seleccion = tree.focus() 
            if not seleccion:
                messagebox.showwarning("Atención", "Selecciona un cliente de la tabla para editar.", parent=top)
                return
            
            item_data = tree.item(seleccion)
            cliente_db_id = item_data['values'][0]
            self._abrir_ventana_edicion(cliente_db_id)
            ejecutar_busqueda()

        edit_button = ttk.Button(search_frame, text="Editar Seleccionado", command=editar_seleccion_tabla)
        edit_button.pack(side=tk.RIGHT, padx=5)
        tree.bind("<Double-1>", lambda e: editar_seleccion_tabla())
        poblar_tabla()
        search_entry.focus()

    def editar_cliente_desde_lista(self):
        seleccion = self.listbox_clientes.curselection()
        if not seleccion:
            messagebox.showwarning("Atención", "Selecciona un cliente de la lista para editar.", parent=self.root)
            return
        
        try:
            # El ID de la BD (PK) está al inicio del string de la listbox
            cliente_id = int(self.listbox_clientes.get(seleccion[0]).split(" | ")[0])
            self._abrir_ventana_edicion(cliente_id)
        except (ValueError, IndexError):
                messagebox.showerror("Error", "No se pudo obtener la ID del cliente seleccionado.", parent=self.root)

    def _abrir_ventana_edicion(self, cliente_id):
        cliente = next((c for c in self.all_clientes_data if c[0] == cliente_id), None)
        if not cliente:
            messagebox.showerror("Error", f"No se encontró el cliente (ID: {cliente_id}) en los datos locales.", parent=self.root)
            return
        top = tk.Toplevel(self.root)
        top.title(f"Editar: {cliente[1] or f'Local #{cliente[7]}'} [DB_ID: {cliente[0]}]")
        top.geometry("350x450")
        fields = {
            "ID Local (Excel):": cliente[7], # destination_number
            "Nombre (Opcional):": cliente[1], # nombre
            "Longitud (Lon):": cliente[2],    # lon
            "Latitud (Lat):": cliente[3],     # lat
            "Días Reparto (,)": cliente[8]     # dias_entrega
        }
        entries = {}
        for field, value in fields.items():
            frm = ttk.Frame(top); frm.pack(fill='x', padx=10, pady=5)
            lbl = ttk.Label(frm, text=field, width=20); lbl.pack(side='left')
            entry = ttk.Entry(frm); entry.insert(0, value if value is not None else ""); entry.pack(side='right', expand=True, fill='x')
            entries[field] = entry

        # Combobox para Tipo de Camión (t_carrier)
        frm_tipo = ttk.Frame(top); frm_tipo.pack(fill='x', padx=10, pady=5)
        lbl_tipo = ttk.Label(frm_tipo, text="Tipo Camión Req.:", width=20); lbl_tipo.pack(side='left')
        combo_tipo = ttk.Combobox(frm_tipo, values=["Largo", "Corto", "Dolly"], state="readonly")
        
        tipo_camion_actual = cliente[4] # t_carrier
        if tipo_camion_actual in combo_tipo['values']:
            combo_tipo.set(tipo_camion_actual)
        else:
            combo_tipo.current(0)
            
        combo_tipo.pack(side='right', expand=True, fill='x')

        frm_exclusivo = ttk.Frame(top); frm_exclusivo.pack(fill='x', padx=10, pady=10)
        exclusivo_var = tk.BooleanVar()
        exclusivo_var.set(bool(cliente[10])) 
        
        check_exclusivo = ttk.Checkbutton(frm_exclusivo, text=" Ruta Exclusiva (Cliente solo)", variable=exclusivo_var)
        check_exclusivo.pack(side='left')

        def guardar_cambios():
            if not messagebox.askyesno("Confirmar", "¿Guardar los cambios?", parent=top): return
            try:
                dest_id = int(entries["ID Local (Excel):"].get().strip())
                nombre = entries["Nombre (Opcional):"].get().strip()
                lon = float(entries["Longitud (Lon):"].get().strip())
                lat = float(entries["Latitud (Lat):"].get().strip())
                dias = entries["Días Reparto (,)"].get().strip()
                tipo = combo_tipo.get()
                exclusivo = 1 if exclusivo_var.get() else 0
                update_cliente(cliente_id, nombre, lon, lat, tipo, dest_id, dias, exclusivo)
                
                messagebox.showinfo("Éxito", "Cliente actualizado.", parent=top)
                top.destroy()
                self.refrescar_datos_locales_y_ui()

            except ValueError:
                messagebox.showerror("Error de Formato", "ID Local, Longitud y Latitud deben ser números válidos.", parent=top)
            except sqlite3.IntegrityError:
                    messagebox.showerror("Error de Duplicado", "Ya existe otro cliente con ese ID de Local.", parent=top)

        ttk.Button(top, text="Guardar Cambios", command=guardar_cambios, style="Accent.TButton").pack(pady=20)

    def eliminar_cliente(self):
        seleccion = self.listbox_clientes.curselection()
        if not seleccion:
            messagebox.showwarning("Atención", "Selecciona al menos un cliente para eliminar."); return
        if messagebox.askyesno("Confirmar", f"¿Seguro que deseas eliminar {len(seleccion)} cliente(s)?"):
            for idx in reversed(seleccion):
                cliente_id = int(self.listbox_clientes.get(idx).split(" | ")[0])
                delete_cliente(cliente_id)
            messagebox.showinfo("Éxito", "Cliente(s) eliminado(s).")
            self.refrescar_datos_locales_y_ui()

    def mostrar_camiones(self):
        top = tk.Toplevel(self.root); top.title("Gestión de Camiones"); top.geometry("450x300")
        tree = ttk.Treeview(top, columns=("ID", "Patente", "Tipo"), show="headings")
        tree.heading("ID", text="ID"); tree.column("ID", width=50)
        tree.heading("Patente", text="Patente"); tree.column("Patente", width=150, anchor="center")
        tree.heading("Tipo", text="Tipo"); tree.column("Tipo", width=150, anchor="center")
        tree.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        def refrescar():
            tree.delete(*tree.get_children())
            for row in get_all_camiones():
                tree.insert("", "end", values=row)
        
        refrescar()

    def mostrar_historial_rutas(self):
        top = tk.Toplevel(self.root); top.title("Historial de Rutas"); top.geometry("700x400")
        tree = ttk.Treeview(top, columns=("Fecha", "Patente", "Clientes"), show="headings")
        tree.heading("Fecha", text="Fecha"); tree.column("Fecha", width=150)
        tree.heading("Patente", text="Patente"); tree.column("Patente", width=100)
        tree.heading("Clientes", text="Clientes"); tree.column("Clientes", width=400)
        tree.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        for fecha, patente, clientes_str in get_historial_rutas():
            if clientes_str:
                clientes_ids = [int(cid) for cid in clientes_str.split(',')]
                display_names = []
                for cid in clientes_ids:
                    cliente_data = next((c for c in self.all_clientes_data if c[0] == cid), None)
                    if cliente_data:
                        display_name = cliente_data[1] if cliente_data[1] else f"Local #{cliente_data[7]}"
                        display_names.append(display_name)
                    patente_display = patente if patente else "Sin Asignar"
                    tree.insert("", "end", values=(fecha, patente_display, ", ".join(display_names)))

    def iniciar_calculo(self):
        patente = self.entry_patente.get().strip().upper()        
        tipo_camion = self.combo_tipo.get()
        camion_id = None

        if patente: # <-- Solo si el usuario escribió una patente
            if not validar_patente(patente):
                messagebox.showwarning("Entrada Inválida", "Formato de patente no válido.", parent=self.root); return
            
            camion_id = get_camion_id_by_patente(patente)
            
            if not camion_id:
                if messagebox.askyesno("Camión Nuevo", f"La patente '{patente}' no existe. ¿Deseas agregarla como un camión tipo '{tipo_camion}'?", parent=self.root):
                    try:
                        camion_id = add_camion(patente, tipo_camion)
                    except sqlite3.Error as e:
                        messagebox.showerror("Error DB", f"No se pudo guardar el camión '{patente}': {e}", parent=self.root)
                        return
                else:
                    return # El usuario canceló agregar el camión

        seleccion = self.listbox_clientes.curselection()
        if not seleccion:
            messagebox.showwarning("Selección Vacía", "Debes seleccionar al menos un cliente.", parent=self.root); return
        
        clientes_exclusivos = []
        for i in seleccion:
            try:
                cliente_db_id = int(self.listbox_clientes.get(i).split(" | ")[0])
                
                cliente_data = next((c for c in self.all_clientes_data if c[0] == cliente_db_id), None)
                
                if cliente_data and cliente_data[10] == 1: 
                    clientes_exclusivos.append(cliente_data)
                
            except (ValueError, IndexError):
                pass
        if clientes_exclusivos and len(seleccion) > 1:
            nombre_exclusivo = clientes_exclusivos[0][1] or f"Local #{clientes_exclusivos[0][7]}"
            messagebox.showerror(
                "Error de Regla",
                f"El cliente '{nombre_exclusivo}' (ID: {clientes_exclusivos[0][7]}) está marcado como RUTA EXCLUSIVA.\n\nNo puede ser ruteado junto a otros clientes.",
                parent=self.root
            )
            return 
        clientes_ids = [int(self.listbox_clientes.get(i).split(" | ")[0]) for i in seleccion]
        
        self.btn_calcular.config(state="disabled", text="Calculando...")
        self.salida_texto.config(state="normal"); self.salida_texto.delete("1.0", tk.END)
        self.salida_texto.insert(tk.END, "Procesando la ruta, por favor espera...")
        self.salida_texto.config(state="disabled")

        cd_seleccionado = self.combo_cd.get()
        if not cd_seleccionado or cd_seleccionado not in self.BODEGAS_CENTRALES or not self.BODEGAS_CENTRALES[cd_seleccionado]:
            messagebox.showwarning("Error", f"El CD '{cd_seleccionado}' no tiene coordenadas de inicio (Bodega) definidas en la app.", parent=self.root)
            self.btn_calcular.config(state="normal", text="Calcular Ruta Óptima") # Reactivar el botón
            return
        
        start_coords = self.BODEGAS_CENTRALES[cd_seleccionado]

        thread = threading.Thread(target=self.ejecutar_y_actualizar_gui, args=(camion_id, clientes_ids, start_coords))
        thread.daemon = True
        thread.start()

    def ejecutar_y_actualizar_gui(self, camion_id, clientes_ids, start_coords):
        """
        Ejecuta el cálculo en un hilo y actualiza la GUI con el resultado.
        MODIFICADO: Pasa 'self.all_clientes_data' a proceso_de_calculo.
        """
        try:
            # Llamamos a la función de route_creator con los 4 argumentos
            resultado = proceso_de_calculo(
                camion_id,
                clientes_ids,
                self.all_clientes_data, # <-- ESTE ES EL ARGUMENTO CRÍTICO
                start_coords           # Pasa las coordenadas de inicio
            )

            if 'error' in resultado:
                raise ValueError(resultado['error'])

            map_file = resultado['map_file']
            ruta_texto = resultado['ruta_texto']
            
            self.root.after(0, self.actualizar_gui_con_resultado, resultado, clientes_ids)

        except Exception as e:
            error_message = f"Error al calcular la ruta:\n{e}"
            self.root.after(0, self.actualizar_interfaz_con_error, error_message)
        
        finally:
            pass

    def sugerir_carga(self):
        """
        Selecciona automáticamente clientes de la lista visible
        hasta alcanzar el 'Target Cajas' o un MÁXIMO DE 4 LOCALES.
        """
        try:
            target_cajas = float(self.target_cajas.get())
        except ValueError:
            messagebox.showerror("Error", "El 'Target Cajas' debe ser un número válido.", parent=self.root)
            return

        self.listbox_clientes.selection_clear(0, tk.END)

        cajas_acumuladas = 0
        locales_acumulados = 0 
        items_en_lista = self.listbox_clientes.get(0, tk.END)
        
        box_regex = re.compile(r"-\s*([\d\.]+)\s+cajas$")

        for index, item_text in enumerate(items_en_lista):
            match = box_regex.search(item_text)
            
            if not match:
                continue 

            try:
                cajas_cliente = float(match.group(1))
            except ValueError:
                continue

            if (cajas_acumuladas + cajas_cliente) > target_cajas:
                continue
            if locales_acumulados >= 4:
                break

            cajas_acumuladas += cajas_cliente
            locales_acumulados += 1
            self.listbox_clientes.selection_set(index)
        
        messagebox.showinfo("Sugerencia de Carga", 
                            f"Sugerencia completada.\n\n"
                            f"Locales seleccionados: {locales_acumulados}\n"
                            f"Total seleccionado: {int(cajas_acumuladas)} cajas.", 
                            parent=self.root)

    def actualizar_gui_con_error(self, error_message):
            """
            Actualiza la GUI cuando ocurre un error en el hilo de cálculo.
            Muestra el error en el widget de texto y en un messagebox.
            """
            self.salida_texto.config(state="normal")
            self.salida_texto.delete("1.0", tk.END)
            self.salida_texto.insert(tk.END, error_message)
            self.salida_texto.config(state="disabled")
            messagebox.showerror("Error en el Cálculo", error_message, parent=self.root)

    def on_check_sobrantes_changed(self):
        """
        Se llama al marcar/desmarcar el checkbox de 'sobrantes'.
        Deshabilita los filtros de CD/Zona si está marcado.
        """
        is_checked = self.sobrantes_var.get()
        self.combo_cd.config(state="disabled" if is_checked else "readonly")
        self.combo_zona.config(state="disabled" if is_checked else "readonly")
        self.refrescar_lista_clientes()

    def actualizar_gui_con_resultado(self, resultado, clientes_ids_ruteados):
        """
        Actualiza la GUI con el resultado exitoso y marca los clientes como 'ruteados'.
        (Esta función REEMPLAZA la que tenías antes).
        """
        self.salida_texto.config(state="normal")
        self.salida_texto.delete("1.0", tk.END)
        
        if 'error' in resultado:
            messagebox.showerror("Error en el Cálculo", resultado['error'])
            self.salida_texto.insert(tk.END, f"Error: {resultado['error']}")
        else:
            self.salida_texto.insert(tk.END, resultado['ruta_texto'])
            
            # --- LÓGICA DE SOBRANTES ---
            # Añadir los clientes ruteados a la lista de 'completados'
            self.clientes_ya_ruteados.update(clientes_ids_ruteados)
            # Refrescar la lista principal (esto los quitará de la vista)
            self.refrescar_lista_clientes()
            # --- FIN DE LÓGICA ---
            
            if messagebox.askyesno("Ruta Calculada", "¿Deseas ver el mapa?"):
                try:
                    webbrowser.open(resultado['map_file'])
                except Exception as e:
                    messagebox.showwarning("Advertencia", f"No se pudo abrir el mapa: {e}")
        
        self.salida_texto.config(state="disabled")
        self.btn_calcular.config(state="normal", text="Calcular Ruta Óptima")
        """
        Actualiza la GUI con el resultado exitoso y marca los clientes como 'ruteados'.
        (Esta función REEMPLAZA la que tenías antes).
        """
        self.salida_texto.config(state="normal")
        self.salida_texto.delete("1.0", tk.END)
        
        if 'error' in resultado:
            messagebox.showerror("Error en el Cálculo", resultado['error'])
            self.salida_texto.insert(tk.END, f"Error: {resultado['error']}")
        else:
            self.salida_texto.insert(tk.END, resultado['ruta_texto'])
            
            # --- LÓGICA DE SOBRANTES ---
            # Añadir los clientes ruteados a la lista de 'completados'
            self.clientes_ya_ruteados.update(clientes_ids_ruteados)
            # Refrescar la lista principal (esto los quitará de la vista)
            self.refrescar_lista_clientes()
            # --- FIN DE LÓGICA ---
            
            if messagebox.askyesno("Ruta Calculada", "¿Deseas ver el mapa?"):
                try:
                    webbrowser.open(resultado['map_file'])
                except Exception as e:
                    messagebox.showwarning("Advertencia", f"No se pudo abrir el mapa: {e}")
        
        self.salida_texto.config(state="disabled")
        self.btn_calcular.config(state="normal", text="Calcular Ruta Óptima")