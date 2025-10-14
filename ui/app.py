import tkinter as tk
from tkinter import messagebox, ttk, simpledialog
import threading
import re
import webbrowser
from datetime import datetime
import sqlite3
from ortools.constraint_solver import pywrapcp, routing_enums_pb2

# Importaciones de los otros módulos del proyecto
from db import db_query, db_execute
from excel_parser import procesar_despachos_del_dia
from route_creator import proceso_de_calculo, client


def validar_patente(patente):
    """Valida el formato de una patente chilena."""
    return bool(re.fullmatch(r"[A-Z]{4}\d{2}|[A-Z]{2}\d{4}", patente.upper()))

class App:
    def __init__(self, root, rol):
        self.rol = rol
        self.root = root
        self.root.title("Ruteador de Camiones v0.3")
        self.root.geometry("850x700")
        self.style = ttk.Style()
        self.style.configure(".", font=("Segoe UI", 10))
        
        self.all_clientes_data = []
        self.despachos_pendientes = {}

        self.setup_ui()
        self.refrescar_datos_locales_y_ui()

    def setup_ui(self):
        """Configura la interfaz gráfica principal."""
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky="nsew")
        self.root.grid_columnconfigure(0, weight=1)
        self.root.grid_rowconfigure(0, weight=1)
        main_frame.grid_columnconfigure(0, weight=1)
        main_frame.grid_rowconfigure(1, weight=1)

        # --- Frames Superiores (Carga y Camión) ---
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
        
        # --- Frame de Gestión (Derecha) ---
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

        # --- Frame de Filtros y Zonas (Derecha) ---
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
        ttk.Button(filtros_gestion_frame, text="Agregar Nuevo CD", command=self.agregar_cd).pack(fill='x', pady=2)
        ttk.Button(filtros_gestion_frame, text="Agregar Nueva Zona al CD", command=self.agregar_zona_al_cd).pack(fill='x', pady=2)
        
        # --- Frame Principal de Clientes (Izquierda) ---
        clientes_frame = ttk.LabelFrame(main_frame, text="3. Clientes", padding="10")
        clientes_frame.grid(row=1, column=0, sticky="nsew", padx=5, pady=5)
        clientes_frame.grid_rowconfigure(0, weight=1)
        clientes_frame.grid_columnconfigure(0, weight=1)
        
        scrollbar = ttk.Scrollbar(clientes_frame, orient=tk.VERTICAL)
        self.listbox_clientes = tk.Listbox(clientes_frame, selectmode=tk.MULTIPLE, yscrollcommand=scrollbar.set, background="#fdfdfd", foreground="#1e1e1e", borderwidth=0, highlightthickness=0)
        scrollbar.config(command=self.listbox_clientes.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.listbox_clientes.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # --- Frame de Salida/Resultados (Izquierda) ---
        salida_frame = ttk.LabelFrame(main_frame, text="4. Resultados de la Ruta", padding="10")
        salida_frame.grid(row=2, column=0, sticky="nsew", padx=5, pady=5)
        salida_frame.grid_rowconfigure(0, weight=1)
        salida_frame.grid_columnconfigure(0, weight=1)
        self.salida_texto = tk.Text(salida_frame, height=10, state="disabled", wrap="word", background="#fdfdfd", foreground="#1e1e1e", borderwidth=0, highlightthickness=0)
        self.salida_texto.grid(row=0, column=0, sticky="nsew")

        # --- Botón Principal ---
        self.style.configure("Accent.TButton", font=("Segoe UI", 12, "bold"))
        self.btn_calcular = ttk.Button(main_frame, text="Calcular Ruta Óptima", command=self.iniciar_calculo, style="Accent.TButton")
        self.btn_calcular.grid(row=3, column=0, columnspan=2, pady=10)

    # El resto de los métodos de la clase App (cargar_y_filtrar_despachos, mostrar_historial_rutas, etc.)
    # se mantienen aquí, sin cambios significativos en su lógica interna, solo en cómo llaman a las funciones
    # que ahora están en otros módulos.
    
    def refrescar_datos_locales_y_ui(self):
        """Carga todos los datos de clientes y actualiza los combos de la UI."""
        self.all_clientes_data = db_query("""
            SELECT 
                c.id, c.nombre, c.lon, c.lat, c.tipo_camion, z.cd, z.nombre, c.destination_id, c.dias_reparto
            FROM 
                clientes c 
            JOIN 
                zonas z ON c.zona_id = z.id
        """)
        self.cargar_cds()

    def cargar_cds(self):
        """Puebla el combobox de Centros de Distribución."""
        cds = sorted(list(set(c[5] for c in self.all_clientes_data if c[5])))
        self.combo_cd['values'] = cds
        if cds:
            self.combo_cd.set(cds[0])
        else:
            self.combo_cd.set("")
        self.cargar_zonas_por_cd()

    def cargar_zonas_por_cd(self, event=None):
        """Puebla el combobox de Zonas basado en el CD seleccionado."""
        cd_seleccionado = self.combo_cd.get()
        zonas = sorted(list(set(c[6] for c in self.all_clientes_data if c[5] == cd_seleccionado)))
        self.combo_zona['values'] = zonas
        if zonas:
            self.combo_zona.set(zonas[0])
        else:
            self.combo_zona.set("")
        self.cargar_clientes_por_zona()

    def cargar_clientes_por_zona(self, event=None):
        """Muestra los clientes en la Listbox según el CD y Zona seleccionados."""
        cd = self.combo_cd.get()
        zona = self.combo_zona.get()
        self.listbox_clientes.delete(0, tk.END)
        
        clientes_filtrados = [c for c in self.all_clientes_data if c[5] == cd and c[6] == zona]
        clientes_filtrados.sort(key=lambda x: x[1])

        for cliente in clientes_filtrados:
            self.listbox_clientes.insert(tk.END, f"{cliente[0]} | {cliente[1]}")

    def cargar_y_filtrar_despachos(self):
        """Maneja la carga del Excel y filtra la lista de clientes."""
        self.despachos_pendientes, mensaje = procesar_despachos_del_dia()
        
        if self.despachos_pendientes is None:
            messagebox.showerror("Error", mensaje)
            return
        
        messagebox.showinfo("Proceso finalizado", mensaje)

        dias_semana = ["Lunes", "Martes", "Miércoles", "Jueves", "Viernes", "Sábado", "Domingo"]
        hoy = dias_semana[datetime.now().weekday()]
        
        self.listbox_clientes.delete(0, tk.END)
        
        clientes_para_hoy = []
        for cliente in self.all_clientes_data:
            cliente_id, nombre, _, _, _, _, _, dest_id, dias_reparto = cliente
            # Chequea si el cliente tiene despacho en el Excel Y si el día de reparto coincide.
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

    def agregar_cd(self):
        """Abre un diálogo para agregar un nuevo Centro de Distribución."""
        nuevo_cd = simpledialog.askstring("Agregar CD", "Código del nuevo CD (ej: 6010):", parent=self.root)
        if nuevo_cd:
            zona_inicial = simpledialog.askstring("Agregar Zona", f"Nombre de la zona inicial para el CD {nuevo_cd} (ej: Urbana):", parent=self.root)
            if zona_inicial:
                try:
                    db_execute("INSERT INTO zonas (nombre, cd) VALUES (?, ?)", (zona_inicial, nuevo_cd))
                    messagebox.showinfo("Éxito", "CD y Zona agregados.", parent=self.root)
                    self.refrescar_datos_locales_y_ui()
                except sqlite3.IntegrityError:
                    messagebox.showerror("Error", "Esa combinación de Zona y CD ya existe.", parent=self.root)

    def agregar_zona_al_cd(self):
        """Abre un diálogo para agregar una nueva Zona a un CD existente."""
        cd_seleccionado = self.combo_cd.get()
        if not cd_seleccionado:
            messagebox.showwarning("Atención", "Selecciona un CD primero.", parent=self.root)
            return
        
        nueva_zona = simpledialog.askstring("Agregar Zona", f"Nombre de la nueva zona para el CD {cd_seleccionado}:", parent=self.root)
        if nueva_zona:
            try:
                db_execute("INSERT INTO zonas (nombre, cd) VALUES (?, ?)", (nueva_zona, cd_seleccionado))
                self.refrescar_datos_locales_y_ui()
                self.combo_zona.set(nueva_zona)
            except sqlite3.IntegrityError:
                messagebox.showerror("Error", "Esa combinación de Zona y CD ya existe.", parent=self.root)
    
    def agregar_cliente(self):
        """Abre una ventana para agregar un nuevo cliente."""
        cd, zona = self.combo_cd.get(), self.combo_zona.get()
        if not cd or not zona:
            messagebox.showwarning("Atención", "Debes seleccionar un CD y una Zona.", parent=self.root); return
        
        zona_id_res = db_query("SELECT id FROM zonas WHERE nombre=? AND cd=?", (zona, cd))
        if not zona_id_res:
            messagebox.showerror("Error", "La zona seleccionada no se encontró en la base de datos.", parent=self.root); return
        zona_id = zona_id_res[0][0]

        # Creación de la ventana Toplevel para agregar cliente
        top = tk.Toplevel(self.root); top.title("Agregar Cliente"); top.geometry("350x450")
        
        # Widgets de la ventana
        fields = ["Nombre:", "Longitud (Lon):", "Latitud (Lat):", "ID Destino (Excel):", "Días Reparto (,)"]
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
                nombre = entries["Nombre:"].get().strip()
                lon = float(entries["Longitud (Lon):"].get().strip())
                lat = float(entries["Latitud (Lat):"].get().strip())
                dest_id = int(entries["ID Destino (Excel):"].get().strip())
                dias = entries["Días Reparto (,) "].get().strip()
                tipo = combo_tipo.get()
                
                if not nombre:
                    messagebox.showwarning("Dato Faltante", "El nombre es obligatorio.", parent=top); return

                db_execute("INSERT INTO clientes (nombre, lon, lat, tipo_camion, zona_id, destination_id, dias_reparto) VALUES (?, ?, ?, ?, ?, ?, ?)",
                           (nombre, lon, lat, tipo, zona_id, dest_id, dias))
                messagebox.showinfo("Éxito", "Cliente agregado.", parent=top)
                top.destroy()
                self.refrescar_datos_locales_y_ui()
            except ValueError:
                messagebox.showerror("Error de Formato", "Longitud, Latitud e ID Destino deben ser números.", parent=top)
            except sqlite3.IntegrityError:
                messagebox.showerror("Error de Duplicado", "Ya existe un cliente con ese ID de Destino.", parent=top)

        ttk.Button(top, text="Guardar Cliente", command=guardar, style="Accent.TButton").pack(pady=20)
    
    def editar_cliente(self):
        """Abre una ventana para editar el cliente seleccionado."""
        seleccion = self.listbox_clientes.curselection()
        if not seleccion:
            messagebox.showwarning("Atención", "Selecciona un cliente para editar.", parent=self.root); return
        
        cliente_id = int(self.listbox_clientes.get(seleccion[0]).split(" | ")[0])
        cliente = next((c for c in self.all_clientes_data if c[0] == cliente_id), None)
        if not cliente: return

        top = tk.Toplevel(self.root); top.title(f"Editar: {cliente[1]}"); top.geometry("350x450")
        
        fields = {"Nombre:": cliente[1], "Longitud (Lon):": cliente[2], "Latitud (Lat):": cliente[3], "ID Destino (Excel):": cliente[7], "Días Reparto (,)": cliente[8]}
        entries = {}
        for field, value in fields.items():
            frm = ttk.Frame(top); frm.pack(fill='x', padx=10, pady=5)
            lbl = ttk.Label(frm, text=field, width=20); lbl.pack(side='left')
            entry = ttk.Entry(frm); entry.insert(0, value if value is not None else ""); entry.pack(side='right', expand=True, fill='x')
            entries[field] = entry

        frm_tipo = ttk.Frame(top); frm_tipo.pack(fill='x', padx=10, pady=5)
        lbl_tipo = ttk.Label(frm_tipo, text="Tipo Camión Req.:", width=20); lbl_tipo.pack(side='left')
        combo_tipo = ttk.Combobox(frm_tipo, values=["Largo", "Corto", "Dolly"], state="readonly"); combo_tipo.set(cliente[4]); combo_tipo.pack(side='right', expand=True, fill='x')

        def guardar_cambios():
            if not messagebox.askyesno("Confirmar", "¿Guardar los cambios?", parent=top): return
            try:
                nombre = entries["Nombre:"].get().strip()
                lon = float(entries["Longitud (Lon):"].get().strip())
                lat = float(entries["Latitud (Lat):"].get().strip())
                dest_id = int(entries["ID Destino (Excel):"].get().strip())
                dias = entries["Días Reparto (,)"].get().strip()
                tipo = combo_tipo.get()
                
                db_execute("UPDATE clientes SET nombre=?, lon=?, lat=?, tipo_camion=?, destination_id=?, dias_reparto=? WHERE id=?",
                           (nombre, lon, lat, tipo, dest_id, dias, cliente_id))
                messagebox.showinfo("Éxito", "Cliente actualizado.", parent=top)
                top.destroy()
                self.refrescar_datos_locales_y_ui()

            except ValueError:
                messagebox.showerror("Error de Formato", "Longitud, Latitud e ID Destino deben ser números.", parent=top)
            except sqlite3.IntegrityError:
                 messagebox.showerror("Error de Duplicado", "Ya existe otro cliente con ese ID de Destino.", parent=top)

        ttk.Button(top, text="Guardar Cambios", command=guardar_cambios, style="Accent.TButton").pack(pady=20)
    
    def eliminar_cliente(self):
        """Elimina el/los cliente(s) seleccionado(s)."""
        seleccion = self.listbox_clientes.curselection()
        if not seleccion:
            messagebox.showwarning("Atención", "Selecciona al menos un cliente para eliminar."); return
        if messagebox.askyesno("Confirmar", f"¿Seguro que deseas eliminar {len(seleccion)} cliente(s)?"):
            for idx in reversed(seleccion):
                cliente_id = int(self.listbox_clientes.get(idx).split(" | ")[0])
                db_execute("DELETE FROM clientes WHERE id = ?", (cliente_id,))
            messagebox.showinfo("Éxito", "Cliente(s) eliminado(s).")
            self.refrescar_datos_locales_y_ui()

    def mostrar_camiones(self):
        """Muestra una ventana para gestionar los camiones."""
        top = tk.Toplevel(self.root); top.title("Gestión de Camiones"); top.geometry("450x300")
        tree = ttk.Treeview(top, columns=("ID", "Patente", "Tipo"), show="headings")
        tree.heading("ID", text="ID"); tree.column("ID", width=50)
        tree.heading("Patente", text="Patente"); tree.column("Patente", width=150, anchor="center")
        tree.heading("Tipo", text="Tipo"); tree.column("Tipo", width=150, anchor="center")
        tree.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        def refrescar():
            tree.delete(*tree.get_children())
            for row in db_query("SELECT id, patente, tipo FROM camiones ORDER BY patente"):
                tree.insert("", "end", values=row)
        
        refrescar() # Carga inicial

    def mostrar_historial_rutas(self):
        """Muestra una ventana con el historial de rutas calculadas."""
        top = tk.Toplevel(self.root); top.title("Historial de Rutas"); top.geometry("700x400")
        tree = ttk.Treeview(top, columns=("Fecha", "Patente", "Clientes"), show="headings")
        tree.heading("Fecha", text="Fecha"); tree.column("Fecha", width=150)
        tree.heading("Patente", text="Patente"); tree.column("Patente", width=100)
        tree.heading("Clientes", text="Clientes"); tree.column("Clientes", width=400)
        tree.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        query = """
            SELECT r.fecha, c.patente, r.clientes FROM rutas r
            JOIN camiones c ON r.camion_id = c.id ORDER BY r.fecha DESC
        """
        for fecha, patente, clientes_str in db_query(query):
            if clientes_str:
                clientes_ids = [int(cid) for cid in clientes_str.split(',')]
                nombres = [c[1] for c in self.all_clientes_data if c[0] in clientes_ids]
                tree.insert("", "end", values=(fecha, patente, ", ".join(nombres)))

    def iniciar_calculo(self):
        """Inicia el proceso de cálculo de la ruta en un hilo separado."""
        patente = self.entry_patente.get().strip().upper()
        if not validar_patente(patente):
            messagebox.showwarning("Entrada Inválida", "Formato de patente no válido."); return
        
        tipo_camion = self.combo_tipo.get()
        camion_id = None
        result = db_query("SELECT id FROM camiones WHERE patente = ?", (patente,))
        if result:
            camion_id = result[0][0]
        else:
            if messagebox.askyesno("Camión Nuevo", f"La patente '{patente}' no existe. ¿Deseas agregarla como un camión tipo '{tipo_camion}'?"):
                db_execute("INSERT INTO camiones (patente, tipo) VALUES (?, ?)", (patente, tipo_camion))
                camion_id = db_query("SELECT id FROM camiones WHERE patente = ?", (patente,))[0][0]
            else:
                return

        seleccion = self.listbox_clientes.curselection()
        if not seleccion:
            messagebox.showwarning("Selección Vacía", "Debes seleccionar al menos un cliente."); return
        
        clientes_ids = [int(self.listbox_clientes.get(i).split(" | ")[0]) for i in seleccion]
        
        self.btn_calcular.config(state="disabled", text="Calculando...")
        self.salida_texto.config(state="normal"); self.salida_texto.delete("1.0", tk.END)
        self.salida_texto.insert(tk.END, "Procesando la ruta, por favor espera...")
        self.salida_texto.config(state="disabled")

        # Ejecuta el cálculo en un hilo para no bloquear la UI.
        thread = threading.Thread(target=self.ejecutar_y_actualizar_gui, args=(camion_id, clientes_ids))
        thread.daemon = True
        thread.start()

    def ejecutar_y_actualizar_gui(self, camion_id, clientes_ids):
        """Llama a la función de cálculo y luego actualiza la GUI con el resultado."""
        resultado = proceso_de_calculo(camion_id, clientes_ids, self.all_clientes_data)
        self.root.after(0, self.actualizar_gui_con_resultado, resultado)

    def actualizar_gui_con_resultado(self, resultado):
        """Actualiza la interfaz de usuario con el resultado del cálculo."""
        self.salida_texto.config(state="normal")
        self.salida_texto.delete("1.0", tk.END)
        if 'error' in resultado:
            messagebox.showerror("Error en el Cálculo", resultado['error'])
            self.salida_texto.insert(tk.END, f"Error: {resultado['error']}")
        else:
            self.salida_texto.insert(tk.END, resultado['ruta_texto'])
            if messagebox.askyesno("Ruta Calculada", "¿Deseas ver el mapa?"):
                try:
                    webbrowser.open(resultado['map_file'])
                except Exception as e:
                    messagebox.showwarning("Advertencia", f"No se pudo abrir el mapa: {e}")
        
        self.salida_texto.config(state="disabled")
        self.btn_calcular.config(state="normal", text="Calcular Ruta Óptima")
