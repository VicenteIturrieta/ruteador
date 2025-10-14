import pandas as pd
from tkinter import filedialog

def procesar_despachos_del_dia():
    """
    Abre un diálogo para seleccionar un archivo Excel, lo procesa para encontrar
    despachos 'PICKED' y devuelve un diccionario con los datos.
    """
    filepath = filedialog.askopenfilename(
        title="Selecciona el archivo Excel de despachos",
        filetypes=(("Archivos Excel", "*.xlsx"), ("Todos los archivos", "*.*"))
    )
    if not filepath:
        return None, "No se seleccionó ningún archivo."
    try:
        # Lee la hoja específica del archivo Excel.
        df = pd.read_excel(filepath, sheet_name="Shipping Temuco")
        
        # Filtra por el estado 'PICKED'.
        df_listo = df[df['Status'] == 'PICKED'].copy()
        if df_listo.empty:
            return {}, "No se encontraron despachos con estado 'PICKED'."

        # Procesa y limpia los datos.
        df_listo['Destination #'] = df_listo['Destination'].str.split('|').str[-1].str.strip()
        df_listo.dropna(subset=['Destination #'], inplace=True)
        df_listo = df_listo[df_listo['Destination #'] != '']
        df_listo['Destination #'] = df_listo['Destination #'].astype(int)
        df_listo['Case Count'] = pd.to_numeric(df_listo['Case Count'], errors='coerce').fillna(0)

        # Agrupa por cliente y suma las cajas.
        despachos = df_listo.groupby('Destination #')['Case Count'].sum()
        despachos_dict = despachos.to_dict()

        return despachos_dict, f"Se procesaron {len(despachos_dict)} clientes con un total de {int(sum(despachos_dict.values()))} cajas."

    except KeyError as e:
        return None, f"Error: No se encontró la columna requerida en el Excel: {e}. Revisa el formato."
    except Exception as e:
        return None, f"Error al procesar el archivo Excel: {e}"
