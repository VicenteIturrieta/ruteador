import pandas as pd
from tkinter import filedialog
from tkinter import messagebox

def procesar_despachos_del_dia():
    """
    Abre un diálogo para seleccionar un archivo Excel de planning, lee las pestañas
    'Pendiente' y 'Shipping' para calcular el total de cajas por local.
    """
    filepath = filedialog.askopenfilename(
        title="Selecciona el archivo Excel de Planning",
        filetypes=(("Archivos Excel", "*.xlsx"), ("Todos los archivos", "*.*"))
    )
    if not filepath:
        return None, "No se seleccionó ningún archivo."

    try:
        # Lee el archivo Excel.
        xls = pd.ExcelFile(filepath)
        df_pendiente = None
        df_shipping = None
        
        # Carga las pestañas si existen.
        if 'Pendiente' in xls.sheet_names:
            df_pendiente = pd.read_excel(xls, sheet_name='Pendiente')
        
        if 'Shipping' in xls.sheet_names:
            df_shipping = pd.read_excel(xls, sheet_name='Shipping')

        if df_pendiente is None and df_shipping is None:
             return None, "Error: No se encontraron las pestañas 'Pendiente' o 'Shipping' en el archivo."

        all_dataframes = []

        # Procesa la pestaña 'Pendiente'.
        if df_pendiente is not None:
            required_cols = ["Destination #", "Order Quantity (WHPK)"]
            if all(col in df_pendiente.columns for col in required_cols):
                df_p = df_pendiente[required_cols].copy()
                df_p.rename(columns={"Destination #": "local_id", "Order Quantity (whpk)": "cajas"}, inplace=True)
                all_dataframes.append(df_p)
            else:
                messagebox.showwarning("Advertencia", "No se encontraron las columnas 'Destination #' y/o 'Order Quantity (whpk)' en la pestaña 'Pendiente'.")

        # Procesa la pestaña 'Shipping'.
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

        # Combina los datos de ambas pestañas.
        df_total = pd.concat(all_dataframes, ignore_index=True)

        # Limpia y procesa los datos combinados.
        df_total.dropna(subset=['local_id'], inplace=True)
        df_total['local_id'] = pd.to_numeric(df_total['local_id'], errors='coerce')
        df_total.dropna(subset=['local_id'], inplace=True)
        df_total['local_id'] = df_total['local_id'].astype(int)
        
        df_total['cajas'] = pd.to_numeric(df_total['cajas'], errors='coerce').fillna(0)

        # Agrupa por local y suma las cajas.
        despachos = df_total.groupby('local_id')['cajas'].sum()
        despachos_dict = despachos.to_dict()

        total_cajas = int(sum(despachos_dict.values()))
        total_locales = len(despachos_dict)
        
        return despachos_dict, f"Se procesaron {total_locales} locales con un total de {total_cajas} cajas."

    except Exception as e:
        return None, f"Error al procesar el archivo Excel: {e}"

