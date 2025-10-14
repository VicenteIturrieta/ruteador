import tkinter as tk
import sv_ttk
from ui.app import App
from login import Login
from db import setup_database
from route_creator import get_api_key

def main():
    """
    Función principal para configurar la base de datos y lanzar la aplicación.
    """
    setup_database()

    api_key = get_api_key()
    if not api_key:
        print("Error: No se puede iniciar la aplicación sin una clave de API válida en config.ini.")
        return

    login_root = tk.Tk()
    sv_ttk.set_theme("light")
    login_app = Login(login_root)
    login_root.mainloop()
    if login_app.rol_usuario:
        main_root = tk.Tk()
        sv_ttk.set_theme("light")
        app = App(main_root, login_app.rol_usuario)
        main_root.mainloop()

if __name__ == "__main__":
    main()
