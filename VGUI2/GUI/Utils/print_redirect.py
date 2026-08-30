import sys
import tkinter as tk

class PrintRedirector:
    def __init__(self, widget: tk.Text, tag: str = "stdout", echo_to: object = None):
        self.widget = widget
        self.tag = tag
        self.echo_to = echo_to

    def write(self, message: str) -> None:
        if not message:
            return
        
        if self.echo_to is not None:
            self.echo_to.write(message)
        
        self.widget.after(0, self._append, message)

    def _append(self, message: str) -> None:
        self.widget.config(state = "normal")
        self.widget.insert("end", message, self.tag)
        self.widget.see("end")
        self.widget.config(state = "disabled")

    def flush(self) -> None:
        if self.echo_to is not None:
            self.echo_to.flush()