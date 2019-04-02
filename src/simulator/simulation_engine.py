
import tkinter as tk
#from tkinter import ttk
#from tkinter import filedialog


class Interface():
    def __init__(self,appname):
        self.window = tk.Tk()
        self.window.title(str(appname))
        self.window.minsize(800, 600)
        self.window.maxsize(1280, 960)
        self.create_widgets()


    def create_widgets(self):
        self.window.mainloop()
        pass


a = Interface("testApp")