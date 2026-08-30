import tkinter as tk
from tkinter import ttk, messagebox
import threading
import time
from typing import Callable, List, Optional, Tuple

from GUI.base_frame import BaseFrame
from Data.themes import THEMES
from GUI.Utils.settings import load_settings, save_settings
from GUI.Frames.map_frame import BASEMAP_OPTIONS, BASE_TILE_SERVERS, MapFrame
from GUI.Utils.map_downloader import MapDownloader, estimate_tiles, get_db_size, clear_db

OVERLAY_OPTIONS = ["None", "Speed", "Heading", "Both"]

DOWNLOAD_ZOOM_MIN, DOWNLOAD_ZOOM_MAX = 1, 19
DEFAULT_DOWNLOAD_ZOOM_LO, DEFAULT_DOWNLOAD_ZOOM_HI = 14, 19
PROGRESS_UI_STEPS = 200 # Cap on how many times the progress bar is refreshed per download

def _format_bytes(num_bytes: int) -> str:
    size = float(num_bytes)
    for unit in ("B", "KB", "MB", "GB"):
        if unit == "B":
            if size < 1024:
                return f"{int(size)} B"
        elif size < 1024 or unit == "GB":
            return f"{size:.1f} {unit}"
        size /= 1024
    return f"{size:.1f} GB"

def _format_duration(seconds: float) -> str:
    seconds = max(0, int(seconds))
    hours, rem = divmod(seconds, 3600)
    minutes, secs = divmod(rem, 60)

    if hours:
        return f"{hours}h {minutes:02d}m"
    if minutes:
        return f"{minutes}m {secs:02d}s"
    return f"{secs}s"

class SettingsFrame(BaseFrame):
    def __init__(self, parent, theme, ctrl, map_frame: MapFrame, on_close: Callable[[], None],
                 on_theme_change: Callable[[str], None],
                 on_overlay_change: Callable[[str], None]):
        self.map_frame = map_frame
        self.on_close = on_close
        self.on_theme_change = on_theme_change
        self.on_overlay_change = on_overlay_change

        self.settings = load_settings()

        self._download_points: List[Tuple[float, float]] = []
        self._downloader: Optional[MapDownloader] = None
        self._download_progress_step = 1
        self._download_start_time = 0.0
        self._closed = False

        super().__init__(parent, theme, ctrl)

    def build(self):
        self.frame = tk.LabelFrame(
            self.parent, text = "Settings",
            font = ("Segoe UI", 10, "bold"), padx = 6, pady = 6,
            bg = self.theme["panel_bg"], fg = self.theme["fg"])
        self.frame.pack(fill = "both", expand = True)

        header = tk.Frame(self.frame, bg = self.theme["panel_bg"])
        header.pack(fill = "x", pady = (0, 6))
        self._header = header

        ttk.Button(header, text = "Close", command = self._on_close_click).pack(side = "right")

        self.general_box = self._make_section(self.frame, "GENERAL")

        self.theme_row = self._make_row(self.general_box)
        self.theme_label = self._make_label(self.theme_row, "Default Theme")
        self.theme_label.pack(side = "left")

        self.theme_var = tk.StringVar(value = self.settings.get("theme", "dark"))
        self.theme_var.trace_add("write", self._on_field_change)
        self.theme_combo = ttk.Combobox(
            self.theme_row, textvariable = self.theme_var, values = list(THEMES.keys()),
            state = "readonly", width = 10)
        self.theme_combo.pack(side = "right")
        self.theme_combo.bind("<<ComboboxSelected>>", self._on_theme_pick)

        self.port_row = self._make_row(self.general_box)
        self.port_label = self._make_label(self.port_row, "Default Serial Port")
        self.port_label.pack(side = "left")

        self.port_var = tk.StringVar(value = self.settings.get("port", "COM4"))
        self.port_var.trace_add("write", self._on_field_change)
        self.port_entry = ttk.Entry(self.port_row, textvariable = self.port_var, width = 10)
        self.port_entry.pack(side = "right")

        self.map_box = self._make_section(self.frame, "MAP")

        self.overlay_row = self._make_row(self.map_box)
        self.overlay_label = self._make_label(self.overlay_row, "Nav Overlay")
        self.overlay_label.pack(side = "left")

        self.overlay_var = tk.StringVar(value = self.settings.get("overlay", "Both"))
        self.overlay_var.trace_add("write", self._on_field_change)
        self.overlay_combo = ttk.Combobox(
            self.overlay_row, textvariable = self.overlay_var, values = OVERLAY_OPTIONS,
            state = "readonly", width = 10)
        self.overlay_combo.pack(side = "right")
        self.overlay_combo.bind("<<ComboboxSelected>>", self._on_overlay_pick)

        self.download_box = self._make_section(self.frame, "MAP TILE DOWNLOAD")
        self._build_download_section(self.download_box)

        self.storage_box = self._make_section(self.frame, "OFFLINE DATA")
        self._build_storage_section(self.storage_box)

        self.bottom_bar = tk.Frame(self.frame, bg = self.theme["panel_bg"])
        self.bottom_bar.pack(side = "bottom", fill = "x", pady = (8, 0))

        self.status_label = tk.Label(
            self.bottom_bar, text = "", font = ("Segoe UI", 9),
            bg = self.theme["panel_bg"], fg = self.theme["green"])
        self.status_label.pack(side = "top", anchor = "w", pady = (0, 4))

        self.save_frame = tk.Frame(
            self.bottom_bar, bg = self.theme["panel_bg"],
            highlightthickness = 2, highlightbackground = self.theme["panel_bg"])
        self.save_frame.pack(side = "top", fill = "x")

        self.save_button = ttk.Button(self.save_frame, text = "Save", command = self._on_save)
        self.save_button.pack(fill = "x")

        self._update_unsaved_indicator()

    def _make_section(self, parent, title: str) -> tk.Frame:
        box = tk.Frame(parent, bg = self.theme["panel_bg"], bd = 1,
                       relief = "solid", highlightbackground = self.theme["border"])
        box.pack(fill = "x", pady = (0, 8))

        header_row = tk.Frame(box, bg = self.theme["panel_bg"])
        header_row.pack(fill = "x", padx = 6, pady = (5, 3))

        title_label = tk.Label(header_row, text = title, font = ("Segoe UI", 8, "bold"),
                               bg = self.theme["panel_bg"], fg = self.theme["fg_dim"])
        title_label.pack(side = "left")

        box._header_row = header_row
        box._title_label = title_label

        return box

    def _make_row(self, parent: tk.Frame) -> tk.Frame:
        row = tk.Frame(parent, bg = self.theme["panel_bg"])
        row.pack(fill = "x", padx = 6, pady = (0, 6))

        return row

    def _make_label(self, parent: tk.Frame, text: str) -> tk.Label:
        return tk.Label(parent, text = text, font = ("Segoe UI", 10),
                        bg = self.theme["panel_bg"], fg = self.theme["fg"])

    def _build_download_section(self, parent: tk.Frame) -> None:
        self.dl_type_row = self._make_row(parent)
        self.dl_type_label = self._make_label(self.dl_type_row, "Map Type")
        self.dl_type_label.pack(side = "left")

        self.dl_type_var = tk.StringVar(value = "Satellite")
        self.dl_type_combo = ttk.Combobox(
            self.dl_type_row, textvariable = self.dl_type_var, values = BASEMAP_OPTIONS,
            state = "readonly", width = 10)
        self.dl_type_combo.pack(side = "right")

        self.dl_zoom_row = self._make_row(parent)
        self.dl_zoom_label = self._make_label(self.dl_zoom_row, "Zoom Levels")
        self.dl_zoom_label.pack(side = "left")

        self.dl_zoom_frame = tk.Frame(self.dl_zoom_row, bg = self.theme["panel_bg"])
        self.dl_zoom_frame.pack(side = "right")

        self.dl_zoom_lo_var = tk.IntVar(value = DEFAULT_DOWNLOAD_ZOOM_LO)
        self.dl_zoom_lo_var.trace_add("write", self._on_download_params_change)
        self.dl_zoom_lo_spin = ttk.Spinbox(
            self.dl_zoom_frame, from_ = DOWNLOAD_ZOOM_MIN, to = DOWNLOAD_ZOOM_MAX,
            textvariable = self.dl_zoom_lo_var, width = 3)
        self.dl_zoom_lo_spin.pack(side = "left")

        self.dl_zoom_to_label = tk.Label(
            self.dl_zoom_frame, text = "to", font = ("Segoe UI", 9),
            bg = self.theme["panel_bg"], fg = self.theme["fg_dim"])
        self.dl_zoom_to_label.pack(side = "left", padx = 4)

        self.dl_zoom_hi_var = tk.IntVar(value = DEFAULT_DOWNLOAD_ZOOM_HI)
        self.dl_zoom_hi_var.trace_add("write", self._on_download_params_change)
        self.dl_zoom_hi_spin = ttk.Spinbox(
            self.dl_zoom_frame, from_ = DOWNLOAD_ZOOM_MIN, to = DOWNLOAD_ZOOM_MAX,
            textvariable = self.dl_zoom_hi_var, width = 3)
        self.dl_zoom_hi_spin.pack(side = "left")

        self.dl_area_row = self._make_row(parent)
        self.dl_select_btn = ttk.Button(
            self.dl_area_row, text = "Select Area on Map", command = self._on_select_area_click)
        self.dl_select_btn.pack(side = "left")

        self.dl_clear_btn = ttk.Button(self.dl_area_row, text = "Clear", command = self._on_clear_area_click)
        self.dl_clear_btn.pack(side = "left", padx = (6, 0))

        self.dl_info_row = self._make_row(parent)
        self.dl_info_label = tk.Label(
            self.dl_info_row, text = "No area selected", font = ("Segoe UI", 9), justify = "left",
            wraplength = 260, bg = self.theme["panel_bg"], fg = self.theme["fg_dim"])
        self.dl_info_label.pack(side = "left", fill = "x")

        self.dl_progress_row = self._make_row(parent)
        self.dl_progress = ttk.Progressbar(self.dl_progress_row, mode = "determinate")

        self.dl_btn_row = self._make_row(parent)
        self.dl_download_btn = ttk.Button(
            self.dl_btn_row, text = "Download", command = self._on_download_click, state = "disabled")
        self.dl_download_btn.pack(side = "left")

        self.dl_cancel_btn = ttk.Button(
            self.dl_btn_row, text = "Cancel", command = self._on_cancel_download_click)

    def _build_storage_section(self, parent: tk.Frame) -> None:
        self.dl_storage_row = self._make_row(parent)
        self.dl_storage_label = self._make_label(self.dl_storage_row, "Offline Data Size")
        self.dl_storage_label.pack(side = "left")

        self.dl_storage_size_label = tk.Label(
            self.dl_storage_row, text = "", font = ("Segoe UI", 9),
            bg = self.theme["panel_bg"], fg = self.theme["fg_dim"])
        self.dl_storage_size_label.pack(side = "right")

        self.dl_empty_row = self._make_row(parent)
        self.dl_empty_btn = ttk.Button(
            self.dl_empty_row, text = "Delete Map Cache", command = self._on_empty_db_click)
        self.dl_empty_btn.pack(side = "left")

        self._refresh_db_size()

    # Area selection
    def _on_select_area_click(self) -> None:
        self._download_points = []
        self.map_frame.clear_download_area()
        self.dl_download_btn.config(state = "disabled")

        self.dl_select_btn.config(state = "disabled")
        self.dl_info_label.config(
            text = "Douvble click to select first corner", fg = self.theme["fg_dim"])
        self.map_frame.request_point(self._on_first_point_picked)

    def _on_first_point_picked(self, coords: Tuple[float, float]) -> None:
        self._download_points = [coords]
        self.dl_info_label.config(
            text = "Double click to select the other corner", fg = self.theme["fg_dim"])
        self.map_frame.request_point(self._on_second_point_picked)

    def _on_second_point_picked(self, coords: Tuple[float, float]) -> None:
        self._download_points.append(coords)
        self.dl_select_btn.config(state = "normal")
        self._update_download_area_preview()

    def _on_clear_area_click(self) -> None:
        self._download_points = []
        self.map_frame.request_point(None)
        self.map_frame.clear_download_area()

        self.dl_select_btn.config(state = "normal")
        self.dl_download_btn.config(state = "disabled")
        self.dl_info_label.config(text = "No area selected", fg = self.theme["fg_dim"])

    def _update_download_area_preview(self) -> None:
        top_left, bottom_right = self._download_bounds()
        if top_left is None:
            return

        self.map_frame.show_download_area(top_left, bottom_right)
        self._on_download_params_change()

    def _download_bounds(self) -> Tuple[Optional[Tuple[float, float]], Optional[Tuple[float, float]]]:
        if len(self._download_points) != 2:
            return None, None

        (lat1, lon1), (lat2, lon2) = self._download_points
        top_left = (max(lat1, lat2), min(lon1, lon2))
        bottom_right = (min(lat1, lat2), max(lon1, lon2))

        return top_left, bottom_right

    def _download_zoom_range(self) -> Optional[Tuple[int, int]]:
        try:
            lo, hi = self.dl_zoom_lo_var.get(), self.dl_zoom_hi_var.get()
        except tk.TclError:
            return None

        return min(lo, hi), max(lo, hi)

    def _on_download_params_change(self, *args) -> None:
        top_left, bottom_right = self._download_bounds()
        zoom_range = self._download_zoom_range()

        if top_left is None or zoom_range is None:
            return

        tile_count, approx_bytes = estimate_tiles(top_left, bottom_right, *zoom_range)
        approx_mb = approx_bytes / (1024 * 1024)

        self.dl_info_label.config(
            text = f"Approx. {tile_count:,} tiles (~{approx_mb:.1f} MB)", fg = self.theme["fg_dim"])
        self.dl_download_btn.config(state = "normal")

    def _resolve_download_tile_server(self) -> str:
        server = BASE_TILE_SERVERS[self.dl_type_var.get()]
        return server[self.theme.get("dark_map", False)] if isinstance(server, dict) else server

    # Download
    def _on_download_click(self) -> None:
        if self._downloader is not None:
            return

        top_left, bottom_right = self._download_bounds()
        zoom_range = self._download_zoom_range()

        if top_left is None or zoom_range is None:
            return

        zoom_lo, zoom_hi = zoom_range
        tile_count, _ = estimate_tiles(top_left, bottom_right, zoom_lo, zoom_hi)

        tile_server = self._resolve_download_tile_server()

        self._downloader = MapDownloader(tile_server)
        self._download_progress_step = max(1, tile_count // PROGRESS_UI_STEPS)
        self._download_start_time = time.monotonic()

        self.dl_download_btn.config(state = "disabled")
        self.dl_select_btn.config(state = "disabled")
        self.dl_clear_btn.config(state = "disabled")
        self.dl_cancel_btn.config(state = "normal")
        self.dl_cancel_btn.pack(side = "left", padx = (6, 0))
        self.dl_progress.pack(fill = "x")
        self.dl_progress.config(maximum = max(tile_count, 1), value = 0)
        self.dl_info_label.config(text = f"Downloading 0 / {tile_count:,} tiles", fg = self.theme["fg_dim"])

        thread = threading.Thread(
            target = self._downloader.download,
            args = (top_left, bottom_right, zoom_lo, zoom_hi),
            kwargs = dict(on_progress = self._on_download_progress, on_done = self._on_download_done),
            daemon = True)
        thread.start()

    def _on_cancel_download_click(self) -> None:
        if self._downloader is not None:
            self._downloader.cancel()
            self.dl_cancel_btn.config(state = "disabled")

    def _on_download_progress(self, done: int, total: int) -> None:
        if self._closed or (done % self._download_progress_step != 0 and done != total):
            return

        try:
            self.frame.after(0, self._update_download_progress_ui, done, total)
        except tk.TclError:
            pass

    def _update_download_progress_ui(self, done: int, total: int) -> None:
        try:
            self.dl_progress.config(value = done)

            elapsed = time.monotonic() - self._download_start_time
            if done > 0 and elapsed > 0:
                remaining = elapsed * (total - done) / done
                eta_text = f" \u2014 ~{_format_duration(remaining)} remaining"
            else:
                eta_text = ""

            self.dl_info_label.config(text = f"Downloading {done:,} / {total:,} tiles{eta_text}")
        except tk.TclError:
            pass

    def _on_download_done(self, completed: bool, tiles_saved: int, error: Optional[str]) -> None:
        if self._closed:
            return

        try:
            self.frame.after(0, self._finish_download_ui, completed, tiles_saved, error)
        except tk.TclError:
            pass

    def _finish_download_ui(self, completed: bool, tiles_saved: int, error: Optional[str]) -> None:
        self._downloader = None

        try:
            self.dl_cancel_btn.pack_forget()
            self.dl_progress.pack_forget()
            self.dl_select_btn.config(state = "normal")
            self.dl_clear_btn.config(state = "normal")
            self.dl_download_btn.config(state = "normal")

            self._refresh_db_size()

            if error is not None:
                self.dl_info_label.config(text = f"Download failed: {error}", fg = self.theme["red"])
            elif not completed:
                self.dl_info_label.config(
                    text = f"Cancelled \u2014 {tiles_saved:,} tiles saved so far", fg = self.theme["orange"])
            else:
                self.dl_info_label.config(
                    text = f"Downloaded {tiles_saved:,} tiles", fg = self.theme["green"])
        except tk.TclError:
            pass

    def _refresh_db_size(self) -> None:
        try:
            self.dl_storage_size_label.config(text = _format_bytes(get_db_size()))
        except tk.TclError:
            pass

    def _on_empty_db_click(self) -> None:
        if self._downloader is not None:
            messagebox.showinfo(
                "Download in progress", "Cancel the current download before emptying the cache.")
            return

        size_bytes = get_db_size()
        if size_bytes == 0:
            messagebox.showinfo("Offline Data", "The offline map cache is already empty.")
            return

        self.dl_empty_btn.config(state = "disabled")
        self.dl_storage_size_label.config(text = "Clearing...")

        thread = threading.Thread(target = self._empty_db_worker, daemon = True)
        thread.start()

    def _empty_db_worker(self) -> None:
        error = None
        try:
            clear_db()
        except Exception as e:
            error = str(e)

        try:
            self.frame.after(0, self._finish_empty_db, error)
        except tk.TclError:
            pass

    def _finish_empty_db(self, error: Optional[str]) -> None:
        try:
            self.dl_empty_btn.config(state = "normal")
            self._refresh_db_size()

            if error is not None:
                self.dl_info_label.config(text = f"Failed to empty cache: {error}", fg = self.theme["red"])
            else:
                self.dl_info_label.config(text = "Offline map cache emptied", fg = self.theme["green"])
        except tk.TclError:
            pass

    def _on_theme_pick(self, event = None) -> None:
        self.status_label.config(text = "")
        self.on_theme_change(self.theme_var.get())

    def _on_overlay_pick(self, event = None) -> None:
        self.status_label.config(text = "")
        self.on_overlay_change(self.overlay_var.get())

    def _on_save(self) -> None:
        self.settings["theme"] = self.theme_var.get()
        self.settings["port"] = self.port_var.get().strip() or "COM4"
        self.settings["overlay"] = self.overlay_var.get()

        if save_settings(self.settings):
            self.status_label.config(text = "Saved", fg = self.theme["green"])
            self._update_unsaved_indicator()
            self._on_close_click()
        else:
            self.status_label.config(text = "Failed to save settings", fg = self.theme["red"])
            self._update_unsaved_indicator()

    def _on_field_change(self, *args) -> None:
        self._update_unsaved_indicator()

    def _has_unsaved_changes(self) -> bool:
        return (self.theme_var.get() != self.settings.get("theme", "dark") or
                self.port_var.get().strip() != self.settings.get("port", "COM4") or
                self.overlay_var.get() != self.settings.get("overlay", "Both"))

    def _update_unsaved_indicator(self) -> None:
        if self._has_unsaved_changes():
            self.save_frame.config(highlightbackground = self.theme["orange"])
            self.save_button.config(text = "Save Changes")
        else:
            self.save_frame.config(highlightbackground = self.theme["panel_bg"])
            self.save_button.config(text = "Save")

    def _on_close_click(self) -> None:
        self._closed = True

        if self._downloader is not None:
            self._downloader.cancel()

        self.map_frame.request_point(None)
        self.map_frame.clear_download_area()

        self.on_close()

    def apply_theme(self, theme: dict) -> None:
        super().apply_theme(theme)
        self.frame.config(bg = theme["panel_bg"], fg = theme["fg"])
        self._header.config(bg = theme["panel_bg"])
        self.bottom_bar.config(bg = theme["panel_bg"])
        self.save_frame.config(bg = theme["panel_bg"])

        self.general_box.config(bg = theme["panel_bg"], highlightbackground = theme["border"])
        self.general_box._header_row.config(bg = theme["panel_bg"])
        self.general_box._title_label.config(bg = theme["panel_bg"], fg = theme["fg_dim"])

        self.map_box.config(bg = theme["panel_bg"], highlightbackground = theme["border"])
        self.map_box._header_row.config(bg = theme["panel_bg"])
        self.map_box._title_label.config(bg = theme["panel_bg"], fg = theme["fg_dim"])

        self.download_box.config(bg = theme["panel_bg"], highlightbackground = theme["border"])
        self.download_box._header_row.config(bg = theme["panel_bg"])
        self.download_box._title_label.config(bg = theme["panel_bg"], fg = theme["fg_dim"])

        self.storage_box.config(bg = theme["panel_bg"], highlightbackground = theme["border"])
        self.storage_box._header_row.config(bg = theme["panel_bg"])
        self.storage_box._title_label.config(bg = theme["panel_bg"], fg = theme["fg_dim"])

        for row in (self.theme_row, self.port_row, self.overlay_row, self.dl_type_row, self.dl_zoom_row,
                    self.dl_zoom_frame, self.dl_area_row, self.dl_info_row, self.dl_progress_row, self.dl_btn_row,
                    self.dl_storage_row, self.dl_empty_row):
            row.config(bg = theme["panel_bg"])

        self.theme_label.config(bg = theme["panel_bg"], fg = theme["fg"])
        self.port_label.config(bg = theme["panel_bg"], fg = theme["fg"])
        self.overlay_label.config(bg = theme["panel_bg"], fg = theme["fg"])
        self.status_label.config(bg = theme["panel_bg"])

        self.dl_type_label.config(bg = theme["panel_bg"], fg = theme["fg"])
        self.dl_zoom_label.config(bg = theme["panel_bg"], fg = theme["fg"])
        self.dl_zoom_to_label.config(bg = theme["panel_bg"], fg = theme["fg_dim"])
        self.dl_info_label.config(bg = theme["panel_bg"])
        self.dl_storage_label.config(bg = theme["panel_bg"], fg = theme["fg"])
        self.dl_storage_size_label.config(bg = theme["panel_bg"], fg = theme["fg_dim"])

        self._update_unsaved_indicator()