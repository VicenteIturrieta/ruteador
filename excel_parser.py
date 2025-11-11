import pandas as pd
from tkinter import filedialog
from tkinter import messagebox
from db import update_dias_reparto_by_dest_id, cliente_exists_by_dest_id

def procesar_fix_planning():
    filepath = filedialog.askopenfilename(
        title="Selecciona el archivo Excel 'FixPlanning'",
        filetypes=(("Archivos Excel", "*.xlsx"), ("Todos los archivos", "*.*"))
    )
    if not filepath:
        return "Proceso cancelado por el usuario."

    try:
        df = pd.read_excel(filepath, dtype={'Local': str})
        
        mapa_dias = {
            'Lunes': ['Lunes'],
            'Martes': ['Martes'],
            'Miercoles': ['Miercoles', 'Miércoles'], # Acepta ambos
            'Jueves': ['Jueves'],
            'Viernes': ['Viernes'],
            'Sábado': ['Sábado', 'Sabado'] # Acepta ambos
        }

        if 'Local' not in df.columns:
            return "Error: No se encontró la columna 'Local' en el archivo."
        
        columnas_excel = set(df.columns)

        locales_actualizados = 0
        locales_no_encontrados = 0

        for _, row in df.iterrows():
            try:
                if pd.isna(row['Local']):
                    continue
                
                local_id = int(row['Local'])
                
                dias_activos = []
                
                # --- INICIO DE LA MODIFICACIÓN ---
                # Iteramos sobre nuestro mapa de días
                for dia_estandar, posibles_nombres in mapa_dias.items():
                    # Revisamos si alguno de los posibles nombres existe en el Excel
                    for nombre_columna in posibles_nombres:
                        if nombre_columna in columnas_excel and pd.notna(row[nombre_columna]):
                            # Si se encuentra, guardamos el nombre estándar (sin tilde)
                            dias_activos.append(dia_estandar)
                            # Pasamos al siguiente día estándar (ej. Jueves)
                            break 
                # --- FIN DE LA MODIFICACIÓN ---

                dias_str = ",".join(dias_activos)
                
                if cliente_exists_by_dest_id(local_id):
                    update_dias_reparto_by_dest_id(dias_str, local_id)
                    locales_actualizados += 1
                else:
                    locales_no_encontrados += 1

            except ValueError:
                # Omite filas donde 'Local' no sea un número
                continue

        return f"Proceso completado.\nLocales actualizados: {locales_actualizados}\nLocales no encontrados en la BD: {locales_no_encontrados}"

    except Exception as e:
        return f"Error al procesar el archivo FixPlanning: {e}"

def procesar_despachos_del_dia():
    filepath = filedialog.askopenfilename(
        title="Selecciona el archivo Excel de Planning",
        filetypes=(("Archivos Excel", "*.xlsx"), ("Todos los archivos", "*.*"))
    )
    if not filepath:
        return None, "No se seleccionó ningún archivo."

    try:
        xls = pd.ExcelFile(filepath)
        df_pendiente = None
        df_shipping = None
        if 'Pendiente' in xls.sheet_names:
            df_pendiente = pd.read_excel(xls, sheet_name='Pendiente')
        
        if 'Shipping' in xls.sheet_names:
            df_shipping = pd.read_excel(xls, sheet_name='Shipping')

        if df_pendiente is None and df_shipping is None:
             return None, "Error: No se encontraron las pestañas 'Pendiente' o 'Shipping' en el archivo."

        all_dataframes = []

        if df_pendiente is not None:
            required_cols = ["Destination #", "Order Quantity (WHPK)"]
            if all(col in df_pendiente.columns for col in required_cols):
                df_p = df_pendiente[required_cols].copy()
                df_p.rename(columns={"Destination #": "local_id", "Order Quantity (WHPK)": "cajas"}, inplace=True)
                all_dataframes.append(df_p)
            else:
                messagebox.showwarning("Advertencia", "No se encontraron las columnas 'Destination #' y/o 'Order Quantity (whpk)' en la pestaña 'Pendiente'.")

        if df_shipping is not None:
            required_cols = ["Local", "Case Count"]
            if all(col in df_shipping.columns for col in required_cols):
                df_s = df_shipping[required_cols].copy()
                df_s.rename(columns={"Local": "local_id", "Case Count": "cajas"}, inplace=True)
                all_dataframes.append(df_s)
            else:
                messagebox.showwarning("Advertencia", "No se encontraron las columnas 'Local' y/o 'Case Count' en la pestaña 'Shipping'.")
        
        if not all_dataframes:
            return {}, "No se pudo leer información de ninguna pestaña. Revisa las columnas del archivo."

        df_total = pd.concat(all_dataframes, ignore_index=True)

        df_total.dropna(subset=['local_id'], inplace=True)
        df_total['local_id'] = pd.to_numeric(df_total['local_id'], errors='coerce')
        df_total.dropna(subset=['local_id'], inplace=True)
        df_total['local_id'] = df_total['local_id'].astype(int)
        
        df_total['cajas'] = pd.to_numeric(df_total['cajas'], errors='coerce').fillna(0)

        despachos = df_total.groupby('local_id')['cajas'].sum()
        despachos_dict = despachos.to_dict()

        total_cajas = int(sum(despachos_dict.values()))
        total_locales = len(despachos_dict)
        
        return despachos_dict, f"Se procesaron {total_locales} locales con un total de {total_cajas} cajas."

    except Exception as e:
        return None, f"Error al procesar el archivo Excel: {e}"

