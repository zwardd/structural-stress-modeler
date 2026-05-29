import json
import tkinter as tk
from tkinter import filedialog


def save_project_dialog(truss):
    """
    Opens a file save dialog and saves the truss project to JSON.
    All serialization I/O is handled here - no file dialog code in main.py.
    
    Args:
        truss: TrussSystem instance to save
        
    Returns:
        str: Status message for user feedback ("PROJECT SAVED SUCCESSFULLY" or None if cancelled)
    """
    root = tk.Tk()
    root.withdraw()
    
    file_path = filedialog.asksaveasfilename(
        defaultextension=".json",
        filetypes=[("JSON Files", "*.json"), ("All Files", "*.*")]
    )
    
    if not file_path:
        root.destroy()
        return None
    
    try:
        if truss.save_to_file(file_path):
            root.destroy()
            return "PROJECT SAVED SUCCESSFULLY"
        else:
            root.destroy()
            return "FAILED TO SAVE FILE"
    except Exception as e:
        root.destroy()
        return f"SAVE ERROR: {str(e)}"


def load_project_dialog(truss):
    """
    Opens a file load dialog and loads a truss project from JSON.
    All serialization I/O and file dialog operations are handled here.
    
    Args:
        truss: TrussSystem instance to load into
        
    Returns:
        tuple: (success: bool, status_message: str)
               success indicates if load was successful
               status_message is for user feedback
    """
    root = tk.Tk()
    root.withdraw()
    
    file_path = filedialog.askopenfilename(
        filetypes=[("JSON Files", "*.json"), ("All Files", "*.*")]
    )
    
    if not file_path:
        root.destroy()
        return False, None
    
    try:
        if truss.load_from_file(file_path):
            root.destroy()
            return True, "PROJECT LOADED"
        else:
            root.destroy()
            return False, "FAILED TO LOAD FILE"
    except Exception as e:
        root.destroy()
        return False, f"LOAD ERROR: {str(e)}"