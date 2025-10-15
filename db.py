import sqlite3

DB_FILE = "rutas.db"

def db_query(query, params=()):
    with sqlite3.connect(DB_FILE) as conn:
        return conn.cursor().execute(query, params).fetchall()

def db_execute(query, params=()):
    with sqlite3.connect(DB_FILE) as conn:
        conn.cursor().execute(query, params)
        conn.commit()

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
            c.id, c.nombre, c.lon, c.lat, c.tipo_camion, z.cd, z.nombre, c.destination_id, c.dias_reparto
        FROM 
            clientes c 
        LEFT JOIN 
            zonas z ON c.zona_id = z.id
    """)

def add_cliente(nombre, lon, lat, tipo_camion, zona_id, dest_id, dias_reparto):
    db_execute("""
        INSERT INTO clientes (nombre, lon, lat, tipo_camion, zona_id, destination_id, dias_reparto) 
        VALUES (?, ?, ?, ?, ?, ?, ?)
    """, (nombre, lon, lat, tipo_camion, zona_id, dest_id, dias_reparto))

def update_cliente(cliente_id, nombre, lon, lat, tipo_camion, dest_id, dias_reparto):
    db_execute("""
        UPDATE clientes SET nombre=?, lon=?, lat=?, tipo_camion=?, destination_id=?, dias_reparto=? 
        WHERE id=?
    """, (nombre, lon, lat, tipo_camion, dest_id, dias_reparto, cliente_id))

def delete_cliente(cliente_id):
    db_execute("DELETE FROM clientes WHERE id = ?", (cliente_id,))

def update_dias_reparto_by_dest_id(dias_str, dest_id):
    db_execute("UPDATE clientes SET dias_reparto = ? WHERE destination_id = ?", (dias_str, dest_id))
    
def cliente_exists_by_dest_id(dest_id):
    return db_query("SELECT id FROM clientes WHERE destination_id = ?", (dest_id,))

def get_distinct_cds():
    return db_query("SELECT DISTINCT cd FROM zonas ORDER BY cd")

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
    CREATE TABLE IF NOT EXISTS rutas_recurrentes (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        nombre_ruta TEXT UNIQUE NOT NULL,
        clientes_ids TEXT NOT NULL
    )
    """)
    _inicializar_tabla_usuarios()
    print("Base de datos verificada y lista.")