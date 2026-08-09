import tkinter as tk
from typing import Any, Callable, Optional

class SwapContainer:
    def __init__(self, parent: tk.Widget, theme: dict, bg: Optional[str] = None):
        self.parent = parent
        self.theme = theme
        self._bg = bg

        self.base_parent = tk.Frame(parent, bg = self._resolve_bg())
        self.base_parent.pack(fill = "both", expand = True)

        self._overlay_parent = tk.Frame(parent, bg = self._resolve_bg())
        self._active_overlay: Optional[Any] = None

    def _resolve_bg(self) -> str:
        return self._bg if self._bg is not None else self.theme["bg"]

    @property
    def is_overlaid(self) -> bool:
        return self._active_overlay is not None

    @property
    def active_overlay(self) -> Optional[Any]:
        return self._active_overlay

    def show_overlay(self, factory: Callable[[tk.Widget], Any]) -> Any:
        if self._active_overlay is not None:
            raise RuntimeError("SwapContainer has already an active overlay")

        self.base_parent.pack_forget()  
        self._overlay_parent.pack(fill = "both", expand = True)

        self._active_overlay = factory(self._overlay_parent)
        return self._active_overlay

    def close_overlay(self) -> None:
        if self._active_overlay is None:
            return

        cleanup = getattr(self._active_overlay, "on_overlay_close", None)
        if callable(cleanup):
            cleanup()

        self._active_overlay.frame.destroy()
        self._active_overlay = None

        self._overlay_parent.pack_forget()
        self.base_parent.pack(fill = "both", expand = True)

    def apply_theme(self, theme: dict) -> None:
        self.theme = theme
        self.base_parent.config(bg = self._resolve_bg())
        self._overlay_parent.config(bg = self._resolve_bg())