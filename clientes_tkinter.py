import tkinter as tk
from tkinter import messagebox, ttk
import sqlite3

DB_FILE = "rutas.db"

def agregar_cliente():
    nombre = entry_nombre.get()
    lon = entry_lon.get()
    lat = entry_lat.get()
    tipo_camion = combo_tipo.get()

    if not nombre or not lon or not lat:
        messagebox.showwarning("Error", "Todos los campos son obligatorios")
        return

    try:
        lon = float(lon)
        lat = float(lat)
    except ValueError:
        messagebox.showwarning("Error", "Longitud y Latitud deben ser números")
        return

    conn = sqlite3.connect(DB_FILE)
    cur = conn.cursor()
    cur.execute("INSERT INTO clientes (nombre, lon, lat, tipo_camion) VALUES (?, ?, ?, ?)",
                (nombre, lon, lat, tipo_camion))
    conn.commit()
    conn.close()

    messagebox.showinfo("Éxito", f"Cliente '{nombre}' agregado")
    entry_nombre.delete(0, tk.END)
    entry_lon.delete(0, tk.END)
    entry_lat.delete(0, tk.END)
    combo_tipo.set("cualquiera")
    cargar_clientes()

def cargar_clientes():
    for i in tree.get_children():
        tree.delete(i)

    conn = sqlite3.connect(DB_FILE)
    cur = conn.cursor()
    cur.execute("SELECT id, nombre, lat, lon, tipo_camion FROM clientes ORDER BY id")
    for row in cur.fetchall():
        tree.insert("", tk.END, values=row)
    conn.close()


# Crear ventana
root = tk.Tk()
root.title("Gestión de Clientes")
root.geometry("650x400")

# --- Formulario para agregar cliente ---
frame_form = tk.Frame(root, padx=10, pady=10)
frame_form.pack(side=tk.TOP, fill=tk.X)

tk.Label(frame_form, text="Nombre:").grid(row=0, column=0, sticky="w")
entry_nombre = tk.Entry(frame_form, width=25)
entry_nombre.grid(row=0, column=1, padx=5)

tk.Label(frame_form, text="Latitud:").grid(row=1, column=0, sticky="w")
entry_lat = tk.Entry(frame_form, width=25)
entry_lat.grid(row=1, column=1, padx=5)

tk.Label(frame_form, text="Longitud:").grid(row=2, column=0, sticky="w")
entry_lon = tk.Entry(frame_form, width=25)
entry_lon.grid(row=2, column=1, padx=5)

tk.Label(frame_form, text="Tipo de camión:").grid(row=3, column=0, sticky="w")
combo_tipo = ttk.Combobox(frame_form, values=["Cualquiera", "Camión largo"], state="readonly")
combo_tipo.current(0)
combo_tipo.grid(row=3, column=1, padx=5)

btn_agregar = tk.Button(frame_form, text="Agregar Cliente", command=agregar_cliente)
btn_agregar.grid(row=4, column=0, columnspan=2, pady=10)

# --- Listado de clientes ---
frame_list = tk.Frame(root, padx=10, pady=10)
frame_list.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

columns = ("ID", "Nombre", "Latitud", "Longitud", "Tipo de camión")
tree = ttk.Treeview(frame_list, columns=columns, show="headings")
for col in columns:
    tree.heading(col, text=col)
    tree.column(col, width=100)
tree.pack(fill=tk.BOTH, expand=True)

cargar_clientes()

root.mainloop()