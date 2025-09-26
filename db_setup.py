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
CREATE TABLE IF NOT EXISTS clientes (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    nombre TEXT NOT NULL,
    lon REAL NOT NULL,
    lat REAL NOT NULL,
    tipo_camion TEXT DEFAULT 'cualquiera'
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

# Insertar clientes iniciales
clientes = [
    ("Inicio", -72.093775, -36.562653, "largo"),
    ("Lider Chillán", -72.105695, -36.615563, "largo"),
    ("Acuenta Chillán", -72.086749, -36.624249, "largo"),
    ("Lider Paul Harris", -72.085021, -36.590386, "largo"),
    ("Acuenta Concepción", -73.03562944645336, -36.793059496945226, "largo"),
    ("Quillón (Acuenta)", -72.47499529063438, -36.74026059015533, "largo"),
    ("Biobio (Lider)", -73.06980746017213, -36.795639735852305, "largo"),
    ("Los Angeles (Lider)", -72.34878480635808, -37.47153730822589, "largo"),
    ("Concepcion Pratt (Lider)", -73.06058291414568, -36.828687144697476, "largo"),
    ("Los Angeles (Express)", -72.36367068422189, -37.47932840254894, "largo")
]
cur.executemany("INSERT INTO clientes (nombre, lon, lat, tipo_camion) VALUES (?, ?, ?, ?)", clientes)

conn.commit()
conn.close()

print("✅ Base de datos creada con clientes iniciales")
