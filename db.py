import sqlite3

DB_FILE = "rutas.db"

def db_query(query, params=()):
    """Ejecuta una consulta SELECT en la base de datos y devuelve los resultados."""
    with sqlite3.connect(DB_FILE) as conn:
        return conn.cursor().execute(query, params).fetchall()

def db_execute(query, params=()):
    """Ejecuta una consulta de modificación (INSERT, UPDATE, DELETE) en la base de datos."""
    with sqlite3.connect(DB_FILE) as conn:
        conn.cursor().execute(query, params)
        conn.commit()

def inicializar_tabla_usuarios():
    """Crea la tabla de usuarios si no existe y agrega un usuario admin por defecto."""
    db_execute("""
        CREATE TABLE IF NOT EXISTS usuarios (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            usuario TEXT UNIQUE NOT NULL,
            password TEXT NOT NULL,
            rol TEXT CHECK(rol IN ('admin','usuario')) NOT NULL
        )
    """)
    # Verifica si el usuario admin ya existe.
    admin_existe = db_query("SELECT * FROM usuarios WHERE usuario = 'admin'")
    if not admin_existe:
        # Si no existe, lo crea.
        db_execute("INSERT INTO usuarios (usuario, password, rol) VALUES (?, ?, ?)",
                   ('admin', 'admin', 'admin'))

def setup_database():
    """
    Configura la base de datos desde cero, creando todas las tablas necesarias
    y poblándolas con datos iniciales.
    """
    conn = sqlite3.connect(DB_FILE)
    cur = conn.cursor()

    # Tablas
    cur.execute("""
    CREATE TABLE IF NOT EXISTS camiones (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        patente TEXT NOT NULL,
        tipo TEXT NOT NULL
    )
    """)

    cur.execute("""
    CREATE TABLE IF NOT EXISTS zonas (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        nombre TEXT NOT NULL,
        cd TEXT NOT NULL,
        UNIQUE(nombre, cd)
    )
    """)

    cur.execute("""
    CREATE TABLE IF NOT EXISTS clientes (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        nombre TEXT,
        lon REAL NOT NULL,
        lat REAL NOT NULL,
        tipo_camion TEXT DEFAULT 'cualquiera',
        zona_id INTEGER,
        destination_id INTEGER UNIQUE NOT NULL,
        dias_reparto TEXT,
        FOREIGN KEY(zona_id) REFERENCES zonas(id)
    )
    """)
    
    cur.execute("""
    CREATE TABLE IF NOT EXISTS rutas (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        camion_id INTEGER,
        fecha TEXT,
        clientes TEXT,
        FOREIGN KEY(camion_id) REFERENCES camiones(id)
    )
    """)

    cur.execute("""
    CREATE TABLE IF NOT EXISTS rutas_recurrentes (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        nombre_ruta TEXT UNIQUE NOT NULL,
        clientes_ids TEXT NOT NULL
    )
    """)

    # Inicializa la tabla de usuarios (admin/admin)
    inicializar_tabla_usuarios()

    conn.commit()
    conn.close()
    print("Base de datos verificada y lista.")

