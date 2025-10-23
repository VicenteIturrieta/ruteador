import sqlite3
import pandas as pd
from tkinter import messagebox,filedialog

DB_FILE = "rutas.db"
def db_query(query, params=()):
    with sqlite3.connect(DB_FILE) as conn:
        return conn.cursor().execute(query, params).fetchall()

def db_execute(query, params=()):
    with sqlite3.connect(DB_FILE) as conn:
        conn.cursor().execute(query, params)
        conn.commit()

def importar_clientes_excel():
    """
    Abre un diálogo para seleccionar el archivo Excel de clientes (nuevo formato),
    lo lee y actualiza la base de datos (INSERT OR REPLACE).
    IGNORA la columna 'dias_entrega'.
    """
    filepath = filedialog.askopenfilename(
        title="Selecciona el archivo Excel de Clientes (con CD, Zona, Formato, etc.)",
        filetypes=(("Archivos Excel", "*.xlsx"), ("Todos los archivos", "*.*"))
    )
    if not filepath:
        return "Importación cancelada por el usuario."

    try:
        df = pd.read_excel(filepath, dtype={'Local': str})
        required_cols = ['CD', 'Local', 'Zona', 'Nombre', 'Formato', 'Dirección', 'Lat', 'Lon', 'T. Carrier']
        missing_cols = [col for col in required_cols if col not in df.columns]
        if missing_cols:
            return f"Error: Faltan las siguientes columnas en el archivo: {', '.join(missing_cols)}"

        clientes_para_db = []
        skipped_rows = 0
        for index, row in df.iterrows():
            try:
                local_id_str = str(row['Local']).strip() 
                if not local_id_str or local_id_str.lower() == 'nan':
                    skipped_rows += 1
                    print(f"Advertencia: Se omitió la fila {index + 2} por valor vacío o inválido en 'Local'.")
                    continue
                local_id = int(float(local_id_str))
                cliente_data = (
                    local_id,              # destination_number
                    row['Nombre'],         # nombre
                    row['Formato'],        # formato
                    row['Dirección'],      # direccion
                    row['CD'],             # cd
                    row['Zona'],           # zona
                    row.get('Lat'),        # lat 
                    row.get('Lon'),        # lon 
                    row['T. Carrier']      # t_carrier
                )
                clientes_para_db.append(cliente_data)
            except (ValueError, TypeError) as e:
                skipped_rows += 1
                print(f"Advertencia: Se omitió la fila {index + 2} por un valor inválido ('Local'='{row.get('Local', 'N/A')}'): {e}")
                continue # Saltar esta fila

        if not clientes_para_db:
             return "No se encontraron filas válidas para importar."
        with sqlite3.connect(DB_FILE) as conn:
            cur = conn.cursor()
            cur.executemany("""
                INSERT OR REPLACE INTO clientes
                (destination_number, nombre, formato, direccion, cd, zona, lat, lon, t_carrier)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
            """, clientes_para_db)
            conn.commit()

        mensaje = f"¡Éxito! Se procesaron {len(clientes_para_db)} clientes."
        if skipped_rows > 0:
            mensaje += f" Se omitieron {skipped_rows} filas por datos inválidos."
        return mensaje

    except ImportError:
         messagebox.showerror("Error de Dependencia", "Las librerías 'pandas' y 'openpyxl' son necesarias. Instálalas con:\npip install pandas openpyxl")
         return "Error: Falta la librería pandas u openpyxl."
    except Exception as e:
        return f"Ocurrió un error inesperado al importar: {e}"

def verificar_credenciales(usuario, password):
    resultado = db_query("SELECT rol FROM usuarios WHERE usuario=? AND password=?", (usuario, password))
    return resultado[0][0] if resultado else None

def _inicializar_tabla_usuarios():
    db_execute("""
        CREATE TABLE IF NOT EXISTS usuarios (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            usuario TEXT UNIQUE NOT NULL,
            password TEXT NOT NULL,
            rol TEXT CHECK(rol IN ('admin','usuario')) NOT NULL
        )
    """)
    admin_existe = db_query("SELECT id FROM usuarios WHERE usuario = 'admin'")
    if not admin_existe:
        db_execute("INSERT INTO usuarios (usuario, password, rol) VALUES (?, ?, ?)",
                   ('admin', 'admin', 'admin'))

def get_all_clientes_with_details():
    return db_query("""
        SELECT
            id, nombre, lon, lat, t_carrier, cd, zona, destination_number, dias_entrega, formato
        FROM
            clientes
        ORDER BY nombre
    """)

def add_cliente(nombre, lon, lat, tipo_camion_ignorado, zona_id_ignorado, dest_id, dias_reparto):
    #como agrego clientes con excel, puede que no se use.
    db_execute("""
        INSERT INTO clientes (nombre, lon, lat, destination_number, dias_entrega)
        VALUES (?, ?, ?, ?, ?)
    """, (nombre, lon, lat, dest_id, dias_reparto))

def update_cliente(cliente_id, nombre, lon, lat, tipo_camion, dest_id, dias_reparto):
    db_execute("""
        UPDATE clientes SET nombre=?, lon=?, lat=?, t_carrier=?, destination_number=?, dias_entrega=? 
        WHERE id=?
    """, (nombre, lon, lat, tipo_camion, dest_id, dias_reparto, cliente_id))

def delete_cliente(cliente_id):
    db_execute("DELETE FROM clientes WHERE id = ?", (cliente_id,))

def update_dias_reparto_by_dest_id(dias_str, dest_id):
    db_execute("UPDATE clientes SET dias_entrega = ? WHERE destination_number = ?", (dias_str, dest_id))
    
def cliente_exists_by_dest_id(dest_id):
    result = db_query("SELECT id FROM clientes WHERE destination_number = ?", (dest_id,))
    return bool(result)

def get_distinct_cds():
    return db_query("SELECT DISTINCT cd FROM zonas ORDER BY cd")

def get_distinct_cds_from_clientes():
    return db_query("SELECT DISTINCT cd FROM clientes WHERE cd IS NOT NULL ORDER BY cd")

def get_zonas_by_cd_from_clientes(cd):
    return db_query("SELECT DISTINCT zona FROM clientes WHERE cd = ? AND zona IS NOT NULL ORDER BY zona", (cd,))

def get_zonas_by_cd(cd):
    return db_query("SELECT nombre FROM zonas WHERE cd = ? ORDER BY nombre", (cd,))

def get_zona_id(nombre_zona, cd):
    resultado = db_query("SELECT id FROM zonas WHERE nombre=? AND cd=?", (nombre_zona, cd))
    return resultado[0][0] if resultado else None

def add_zona(nombre_zona, cd):
    db_execute("INSERT INTO zonas (nombre, cd) VALUES (?, ?)", (nombre_zona, cd))

def get_all_camiones():
    return db_query("SELECT id, patente, tipo FROM camiones ORDER BY patente")

def get_camion_id_by_patente(patente):
    resultado = db_query("SELECT id FROM camiones WHERE patente = ?", (patente,))
    return resultado[0][0] if resultado else None

def add_camion(patente, tipo):
    db_execute("INSERT INTO camiones (patente, tipo) VALUES (?, ?)", (patente, tipo))
    return get_camion_id_by_patente(patente)

def get_historial_rutas():
    return db_query("""
        SELECT r.fecha, c.patente, r.clientes FROM rutas r
        JOIN camiones c ON r.camion_id = c.id ORDER BY r.fecha DESC
    """)

def guardar_ruta(camion_id, fecha, clientes_str):
    db_execute("INSERT INTO rutas (camion_id, fecha, clientes) VALUES (?, ?, ?)", (camion_id, fecha, clientes_str))

def setup_database():
    db_execute("""
    CREATE TABLE IF NOT EXISTS camiones (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        patente TEXT NOT NULL UNIQUE,
        tipo TEXT NOT NULL
    )
    """)
    db_execute("""
    CREATE TABLE IF NOT EXISTS zonas (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        nombre TEXT NOT NULL,
        cd TEXT NOT NULL,
        UNIQUE(nombre, cd)
    )
    """)
    db_execute("""
        CREATE TABLE IF NOT EXISTS clientes (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            destination_number INTEGER UNIQUE NOT NULL, -- Columna 'Local'
            nombre TEXT,
            formato TEXT,       -- Nueva
            direccion TEXT,     -- Nueva
            cd TEXT,            -- Nueva
            zona TEXT,          -- Nueva (antes era zona_id)
            dias_entrega TEXT,  -- Se mantiene, se llenará con FixPlanning
            lat REAL,           -- Nueva o actualizada
            lon REAL,           -- Nueva o actualizada
            t_carrier TEXT      -- Nueva ('T. Carrier')
            -- Se elimina zona_id y tipo_camion (ahora es t_carrier)
        )
    """)
    db_execute("""
    CREATE TABLE IF NOT EXISTS rutas (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        camion_id INTEGER,
        fecha TEXT,
        clientes TEXT,
        FOREIGN KEY(camion_id) REFERENCES camiones(id)
    )
    """)
    db_execute("""
    CREATE TABLE IF NOT EXISTS rutas_favoritas (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        nombre TEXT UNIQUE NOT NULL,
        clientes_ids TEXT NOT NULL)
    """)

    columnas_clientes = ['formato', 'direccion', 'cd', 'zona', 't_carrier', 'dias_entrega', 'lat', 'lon']
    for col in columnas_clientes:
        try:
            tipo_col = 'REAL' if col in ['lat', 'lon'] else 'TEXT'
            db_execute(f"ALTER TABLE clientes ADD COLUMN {col} {tipo_col};")
        except sqlite3.OperationalError: pass 
    try:
        db_execute("ALTER TABLE camiones ADD COLUMN capacidad INTEGER DEFAULT 100;")
    except sqlite3.OperationalError: pass

    _inicializar_tabla_usuarios()

    print("Base de datos lista.")

if __name__ == "__main__":
    setup_database()
    messagebox.showinfo("Base de Datos", "Base de datos configurada correctamente.")