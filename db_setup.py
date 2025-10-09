import sqlite3

def setup_db():
    conn = sqlite3.connect("rutas.db")
    cur = conn.cursor()

    # -------------------
    # Tablas básicas
    # -------------------
    cur.execute("""
    CREATE TABLE IF NOT EXISTS camiones (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        patente TEXT NOT NULL,
        tipo TEXT NOT NULL
    )
    """)
    
    # -------------------
    # Tabla ZONAS (Define el CD (Sede) y la Zona Geográfica)
    # -------------------
    cur.execute("DROP TABLE IF EXISTS zonas") # Borrar la tabla para asegurar la nueva estructura
    
    cur.execute("""
    CREATE TABLE zonas (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        -- 'nombre' es la ZONA GEOGRÁFICA (ej. 'Norte', 'Sur', 'Centro')
        nombre TEXT NOT NULL,
        -- 'cd' es el CÓDIGO de la SEDE/CD (ej. '6010', '6024', '6003')
        cd TEXT NOT NULL
    )
    """)

    # Insertar zonas/áreas geográficas iniciales con códigos CD
    # Nota: Múltiples Zonas pueden pertenecer al mismo CD
    zonas_ejemplo = [
        # Zonas para el CD '6010'
        ("Norte", "6010"), 
        ("Sur", "6010"),
        ("Rural", "6010"),
        # Zonas para el CD '6024'
        ("Centro", "6024"),
        ("Periurbano", "6024"),
        # Zonas para el CD '6003'
        ("Oriente", "6003"),
        ("Poniente", "6003"),
    ]
    # Usamos un identificador compuesto (Zona + CD) para el mapeo
    zonas_para_insercion = [(f"{zona_nombre} ({cd_code})", cd_code) for zona_nombre, cd_code in zonas_ejemplo]
    cur.executemany("INSERT OR IGNORE INTO zonas (nombre, cd) VALUES (?, ?)", zonas_ejemplo)

    # Mapear zonas a IDs (usando solo el nombre de la zona geográfica)
    cur.execute("SELECT id, nombre FROM zonas")
    zona_map = {nombre: zid for zid, nombre in cur.fetchall()}

    # -------------------
    # Tabla clientes
    # -------------------
    cur.execute("DROP TABLE IF EXISTS clientes") # Borrar la tabla para asegurar la nueva estructura

    cur.execute("""
    CREATE TABLE clientes (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        nombre TEXT NOT NULL,
        lon REAL NOT NULL,
        lat REAL NOT NULL,
        tipo_camion TEXT DEFAULT 'cualquiera',
        zona_id INTEGER,
        destination_id INTEGER UNIQUE,
        dias_reparto TEXT,
        FOREIGN KEY(zona_id) REFERENCES zonas(id)
    )
    """)

    # -------------------
    # Insertar clientes iniciales (Asignando a las Zonas Geográficas)
    # -------------------
    clientes = [
        # Asignación Cliente -> ZONA GEOGRÁFICA
        ("Bodega (Inicio)", -72.093775, -36.562653, "largo", "Norte"), 
        ("Lider Chillán", -72.105695, -36.615563, "largo", "Norte"), 
        ("Acuenta Chillán", -72.086749, -36.624249, "largo", "Sur"), 
        ("Lider Paul Harris", -72.085021, -36.590386, "largo", "Sur"), 
        ("Acuenta Concepción", -73.035629, -36.793059, "largo", "Centro"), 
        ("Quillón (Acuenta)", -72.474995, -36.740260, "largo", "Rural"), 
        ("Biobio (Lider)", -73.069807, -36.795639, "largo", "Centro"), 
        ("Los Angeles (Lider)", -72.348784, -37.471537, "largo", "Oriente"), 
        ("Concepcion Pratt (Lider)", -73.060582, -36.828687, "largo", "Periurbano"), 
        ("Los Angeles (Express)", -72.363670, -37.479328, "largo", "Oriente") 
    ]

    clientes_con_zona_id = []
    for nombre, lon, lat, tipo, zona_nombre in clientes:
        if zona_nombre in zona_map:
            clientes_con_zona_id.append((nombre, lon, lat, tipo, zona_map[zona_nombre]))
        else:
            print(f"⚠️ Advertencia: Zona '{zona_nombre}' no encontrada en el mapeo para el cliente {nombre}")


    cur.executemany("""
        INSERT INTO clientes (nombre, lon, lat, tipo_camion, zona_id)
        VALUES (?, ?, ?, ?, ?)
    """, clientes_con_zona_id)

    # -------------------
    # Asignar destination_id automático y días de reparto por defecto
    # -------------------
    cur.execute("SELECT id FROM clientes ORDER BY id")
    ids = [row[0] for row in cur.fetchall()]

    for i, cliente_id in enumerate(ids, start=1):
        cur.execute("""
            UPDATE clientes SET destination_id = ?, dias_reparto = ?
            WHERE id = ?
        """, (i, "Lunes,Miércoles,Viernes", cliente_id))
    
    # -------------------
    # Tablas rutas y rutas recurrentes (sin cambios)
    # -------------------
    cur.execute("""
    CREATE TABLE IF NOT EXISTS rutas (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        camion_id INTEGER,
        fecha TEXT,
        clientes TEXT,  -- lista de IDs en orden
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

    conn.commit()
    conn.close()
    print("✅ Base de datos creada y lista para usar en cualquier equipo")

if __name__ == "__main__":
    setup_db()