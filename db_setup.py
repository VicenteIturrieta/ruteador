import sqlite3

conn = sqlite3.connect("rutas.db")
cur = conn.cursor()

# Crear tablas
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
    cd TEXT NOT NULL
)
""")

cur.execute("""
CREATE TABLE IF NOT EXISTS clientes (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    nombre TEXT NOT NULL,
    lon REAL NOT NULL,
    lat REAL NOT NULL,
    tipo_camion TEXT DEFAULT 'cualquiera',
    zona_id INTEGER,
    FOREIGN KEY(zona_id) REFERENCES zonas(id)
)
""")

cur.execute("""
CREATE TABLE IF NOT EXISTS rutas (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    camion_id INTEGER,
    fecha TEXT,
    clientes TEXT,  -- lista de IDs en orden
    FOREIGN KEY(camion_id) REFERENCES camiones(id)
)
""")

# Insertar zonas iniciales
zonas = [
    ("Bodega", "Chillán"),
    ("Lider Chillán", "Chillán"),
    ("Acuenta Chillán", "Chillán"),
    ("Lider Paul Harris", "Chillán"),
    ("Acuenta Concepción", "Concepción"),
    ("Quillón (Acuenta)", "Quillón"),
    ("Biobio (Lider)", "Concepción"),
    ("Los Angeles (Lider)", "Los Ángeles"),
    ("Concepcion Pratt (Lider)", "Concepción"),
    ("Los Angeles (Express)", "Los Ángeles")
]
cur.executemany("INSERT INTO zonas (nombre, cd) VALUES (?, ?)", zonas)

# Mapear zonas a IDs
cur.execute("SELECT id, nombre FROM zonas")
zona_map = {nombre: zid for zid, nombre in cur.fetchall()}

# Insertar clientes iniciales referenciando zona_id
clientes = [
    ("Inicio", -72.093775, -36.562653, "largo", "Bodega"),
    ("Lider Chillán", -72.105695, -36.615563, "largo", "Lider Chillán"),
    ("Acuenta Chillán", -72.086749, -36.624249, "largo", "Acuenta Chillán"),
    ("Lider Paul Harris", -72.085021, -36.590386, "largo", "Lider Paul Harris"),
    ("Acuenta Concepción", -73.035629, -36.793059, "largo", "Acuenta Concepción"),
    ("Quillón (Acuenta)", -72.474995, -36.740260, "largo", "Quillón (Acuenta)"),
    ("Biobio (Lider)", -73.069807, -36.795639, "largo", "Biobio (Lider)"),
    ("Los Angeles (Lider)", -72.348784, -37.471537, "largo", "Los Angeles (Lider)"),
    ("Concepcion Pratt (Lider)", -73.060582, -36.828687, "largo", "Concepcion Pratt (Lider)"),
    ("Los Angeles (Express)", -72.363670, -37.479328, "largo", "Los Angeles (Express)")
]

clientes_con_zona_id = [(nombre, lon, lat, tipo, zona_map[zona]) for nombre, lon, lat, tipo, zona in clientes]

cur.executemany("INSERT INTO clientes (nombre, lon, lat, tipo_camion, zona_id) VALUES (?, ?, ?, ?, ?)", clientes_con_zona_id)

conn.commit()
conn.close()

print("✅ Base de datos creada con zonas y clientes normalizados")
