import sqlite3

DB_FILE = "rutas.db"

# --- FUNCIONES BÁSICAS DE CONEXIÓN ---

def db_query(query, params=()):
    """Ejecuta una consulta (SELECT) y devuelve todos los resultados."""
    with sqlite3.connect(DB_FILE) as conn:
        return conn.cursor().execute(query, params).fetchall()

def db_execute(query, params=()):
    """Ejecuta una operación de escritura (INSERT, UPDATE, DELETE)."""
    with sqlite3.connect(DB_FILE) as conn:
        conn.cursor().execute(query, params)
        conn.commit()

# --- CONFIGURACIÓN INICIAL DE LA BASE DE DATOS ---

def setup_database():
    """
    Asegura que todas las tablas y columnas necesarias existan.
    Esta función es segura de ejecutar múltiples veces y no borrará datos.
    """
    print("Configurando la base de datos...")

    # Tabla de Usuarios
    db_execute("""
        CREATE TABLE IF NOT EXISTS usuarios (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            usuario TEXT UNIQUE NOT NULL,
            password TEXT NOT NULL,
            rol TEXT CHECK(rol IN ('admin','usuario')) NOT NULL
        )
    """)
    # Crear usuario admin por defecto si no existe
    if not db_query("SELECT * FROM usuarios WHERE usuario = 'admin'"):
        db_execute("INSERT INTO usuarios (usuario, password, rol) VALUES (?, ?, ?)",
                   ('admin', 'admin', 'admin'))

    # Tabla de Camiones
    db_execute("""
        CREATE TABLE IF NOT EXISTS camiones (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            patente TEXT UNIQUE NOT NULL,
            tipo TEXT,
            capacidad INTEGER DEFAULT 100
        )
    """)

    # Tabla de Clientes
    db_execute("""
        CREATE TABLE IF NOT EXISTS clientes (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            destination_number INTEGER UNIQUE NOT NULL,
            nombre TEXT,
            lon REAL,
            lat REAL,
            dias_entrega TEXT DEFAULT 'lunes,martes,miercoles,jueves,viernes',
            cd TEXT,
            zona TEXT
        )
    """)

    # Tabla de Rutas (Historial)
    db_execute("""
        CREATE TABLE IF NOT EXISTS rutas (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            camion_id INTEGER,
            fecha TEXT,
            clientes TEXT,
            FOREIGN KEY(camion_id) REFERENCES camiones(id)
        )
    """)

    # Tabla de Rutas Favoritas (antes llamadas recurrentes)
    db_execute("""
        CREATE TABLE IF NOT EXISTS rutas_favoritas (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            nombre TEXT UNIQUE NOT NULL,
            clientes_ids TEXT NOT NULL
        )
    """)
    
    # Intentar añadir columnas que podrían faltar en versiones antiguas de la DB
    # Esto no hace nada si la columna ya existe, por lo que es seguro.
    try:
        db_execute("ALTER TABLE camiones ADD COLUMN capacidad INTEGER DEFAULT 100;")
    except sqlite3.OperationalError:
        pass # La columna ya existía, no hay problema.

    try:
        db_execute("ALTER TABLE clientes ADD COLUMN dias_entrega TEXT DEFAULT 'lunes,martes,miercoles,jueves,viernes';")
    except sqlite3.OperationalError:
        pass # La columna ya existía.

    print("✅ Base de datos lista.")


# --- FUNCIONES DE GESTIÓN (CRUD para el resto de la app) ---

# -- Clientes --
def get_all_clientes():
    """Devuelve una lista de todos los clientes."""
    return db_query("SELECT id, destination_number, nombre, dias_entrega, cd, zona FROM clientes ORDER BY nombre")

def add_cliente(dest_num, nombre, dias_entrega_str):
    """Agrega un nuevo cliente a la base de datos."""
    db_execute("INSERT INTO clientes (destination_number, nombre, dias_entrega) VALUES (?, ?, ?)",
               (dest_num, nombre, dias_entrega_str))

def update_cliente(cliente_id, dest_num, nombre, dias_entrega_str):
    """Actualiza la información de un cliente existente."""
    db_execute("UPDATE clientes SET destination_number = ?, nombre = ?, dias_entrega = ? WHERE id = ?",
               (dest_num, nombre, dias_entrega_str, cliente_id))

def delete_cliente(cliente_id):
    """Elimina un cliente de la base de datos."""
    db_execute("DELETE FROM clientes WHERE id = ?", (cliente_id,))


# -- Camiones --
def get_all_camiones():
    """Devuelve una lista de todos los camiones."""
    return db_query("SELECT id, patente, tipo, capacidad FROM camiones ORDER BY patente")

def add_camion(patente, tipo, capacidad):
    """Agrega un nuevo camión."""
    db_execute("INSERT INTO camiones (patente, tipo, capacidad) VALUES (?, ?, ?)",
               (patente, tipo, capacidad))

# (Puedes agregar funciones update_camion y delete_camion si las necesitas)


# --- Bloque de ejecución ---
if __name__ == "__main__":
    # Esto permite ejecutar 'python db.py' desde la terminal para configurar la base de datos por primera vez.
    setup_database()