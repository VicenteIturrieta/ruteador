import sqlite3

DB_FILE = "rutas.db"

def get_camiones():
    conn = sqlite3.connect(DB_FILE)
    cur = conn.cursor()
    cur.execute("SELECT id, patente, tipo FROM camiones ORDER BY id")
    camiones = cur.fetchall()
    conn.close()
    return camiones
