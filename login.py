import tkinter as tk
from tkinter import ttk, messagebox
from db import db_query

class Login:
    def __init__(self, root):
        self.root = root
        self.root.title("Login - Ruteador de Camiones")
        self.root.geometry("300x200")
        self.root.resizable(False, False)
        
        self.rol_usuario = None # Almacenará 'admin' o 'usuario' si el login es exitoso.

        # Centrar la ventana
        self.root.update_idletasks()
        width = self.root.winfo_width()
        height = self.root.winfo_height()
        x = (self.root.winfo_screenwidth() // 2) - (width // 2)
        y = (self.root.winfo_screenheight() // 2) - (height // 2)
        self.root.geometry(f'{width}x{height}+{x}+{y}')

        main_frame = ttk.Frame(self.root, padding="20")
        main_frame.pack(expand=True, fill="both")

        ttk.Label(main_frame, text="Usuario:").pack()
        self.entry_user = ttk.Entry(main_frame)
        self.entry_user.pack(fill="x", pady=5)
        self.entry_user.focus() # Pone el foco en el campo de usuario

        ttk.Label(main_frame, text="Contraseña:").pack()
        self.entry_pass = ttk.Entry(main_frame, show="*")
        self.entry_pass.pack(fill="x", pady=5)

        # Permite presionar Enter para ingresar
        self.root.bind('<Return>', self.verificar_login)

        ttk.Button(main_frame, text="Ingresar", command=self.verificar_login, style="Accent.TButton").pack(pady=10)

    def verificar_login(self, event=None):
        """Verifica las credenciales del usuario contra la base de datos."""
        usuario = self.entry_user.get().strip()
        password = self.entry_pass.get().strip()
        
        if not usuario or not password:
            messagebox.showerror("Error", "Debes ingresar usuario y contraseña.")
            return

        resultado = db_query("SELECT rol FROM usuarios WHERE usuario=? AND password=?", (usuario, password))
        
        if resultado:
            self.rol_usuario = resultado[0][0]
            self.root.destroy() # Cierra la ventana de login si es exitoso
        else:
            messagebox.showerror("Error", "Usuario o contraseña incorrectos.")
            self.entry_pass.delete(0, 'end') # Limpia el campo de contraseña
