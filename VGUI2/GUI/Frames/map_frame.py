import tkinter as tk
from tkinter import ttk
from typing import Callable, Tuple
from PIL import Image, ImageDraw, ImageTk
import tkintermapview
import os

from GUI.base_frame import BaseFrame

ICON_SIZE = 50
HOME_ICON_SIZE = 20
FALLBACK_POS = (60.1849, 24.8250)

base_path = os.path.join(os.path.dirname(__file__), "..", "..")

class MapFrame(BaseFrame):
    def __init__(self, parent, theme, ctrl,
                 on_add_waypoint: Callable[[Tuple[float, float]], None],
                 on_set_home: Callable[[Tuple[float, float]], None]):
        
        self.on_add_waypoint = on_add_waypoint
        self.on_set_home = on_set_home

        self._v_marker = None
        self._v_icon_cache: dict[int, ImageTk.PhotoImage] = {}
        self._home_marker = None
        self._home_icon = None
        self._map_initialised = False
        self._map_fallback_done = False
        self._last_heading = -1
        self._last_home = (0.0, 0.0)

        try:
            home_icon_path = os.path.join(base_path, "icons", "home_icon.png")
            self._home_icon = tk.PhotoImage(file = home_icon_path)
        except Exception:
            print("Failed to load home icon")

        super().__init__(parent, theme, ctrl)

    def build(self):
        self.frame = tk.LabelFrame(self.parent, text = "Map",
                                   font = ("Segoe UI", 10, "bold"), padx = 6, pady = 6,
                                   bg = self.theme["panel_bg"], fg = self.theme["fg"])
        self.frame.pack(fill = "both", expand = True)

        self.toolbar = tk.Frame(self.frame, bg = self.theme["panel_bg"])
        self.toolbar.pack(fill = "x", pady = (0, 6))

        self.follow_var = tk.BooleanVar(value = True)
        self.chk_follow = tk.Checkbutton(
            self.toolbar, text = "Center on target", variable = self.follow_var,
            font = ("Segoe UI", 9), bg = self.theme["panel_bg"], fg = self.theme["fg"],
            activebackground = self.theme["panel_bg"], activeforeground = self.theme["fg"],
            selectcolor = self.theme["checkbox"],
        )
        self.chk_follow.pack(side = "left", padx = 5)

        ttk.Button(self.toolbar, text = "Snap to target", command = self._center_on_target).pack(side = "left", padx = 6)

        self.widget = tkintermapview.TkinterMapView(self.frame, corner_radius = 4)
        self.widget.pack(fill = "both", expand = True)
        self.widget.set_zoom(16)

        self.widget.add_right_click_menu_command(
            label = "Add Waypoint", command = self.on_add_waypoint, pass_coords = True
        )

        self.widget.add_right_click_menu_command(
            label = "Set Home WP", command = self.on_set_home, pass_coords = True
        )

        self._v_img_orig = self._load_icon()
        if self._home_icon == None:
            self._home_icon = ImageTk.PhotoImage(self._make_home_pin_icon())

    # Icon 
    @staticmethod
    def _load_icon() -> Image.Image:
        try:
            raw = Image.open("icons/vene1.png").convert("RGBA")
        except (FileNotFoundError, OSError):
            raw = MapFrame._make_fallback_icon()

        raw.thumbnail((ICON_SIZE, ICON_SIZE), Image.LANCZOS)
        canvas = Image.new("RGBA", (ICON_SIZE, ICON_SIZE), (0, 0, 0, 0))
        canvas.paste(raw, ((ICON_SIZE - raw.width) // 2, (ICON_SIZE - raw.height) // 2), raw) 
        return canvas
    
    @staticmethod
    def _make_fallback_icon() -> Image.Image:
        s = 48
        img = Image.new("RGBA", (s, s), (0, 0, 0, 0))
        draw = ImageDraw.Draw(img)
        cx = s // 2
        draw.polygon([(cx, 2), (s - 6, s - 8), (cx, s - 16), (6, s - 8)],
                     fill = (31, 111, 235, 220), outline = (240, 246, 252, 255))
        
        return img
    
    @staticmethod
    def _make_home_pin_icon() -> Image.Image:
        s, cx = HOME_ICON_SIZE, HOME_ICON_SIZE // 2
        img = Image.new("RGBA", (s, s), (0, 0, 0, 0))
        draw = ImageDraw.Draw(img)
        draw.ellipse([3, 1, s - 3, s - 3 - (s // 3)], fill=(56, 139, 253, 230))
        draw.polygon([(3, s - 3 - (s // 3)), (s - 3, s - 3 - (s // 3)), (cx, s - 2)], fill = (56, 139, 253, 230))
        draw.ellipse([cx - 4, 5, cx + 4, 13], fill = (255, 255, 255, 210))

        return img
    
    def _get_icon(self, heading: int) -> ImageTk.PhotoImage:
        if heading not in self._v_icon_cache:
            rotated = self._v_img_orig.rotate(-heading, resample = Image.BICUBIC)
            self._v_icon_cache[heading] = ImageTk.PhotoImage(rotated)

        return self._v_icon_cache[heading]
    
    # Marker Helper
    def _replace_marker(self, old_marker, lat, lon, icon, text):
        if old_marker is not None:
            old_marker.delete()

        return self.widget.set_marker(lat, lon, icon = icon, text = text, text_color = self.theme["red"])
    
    # Toolbar Action
    def _center_on_target(self) -> None:
        d = self.ctrl.get_telemetry_data()
        if d["lat"] or d["lon"]:
            self.widget.set_position(d["lat"], d["lon"])

    # Tile Server
    def set_tiles(self, is_dark: bool) -> None:
        if is_dark:
            self.widget.set_tile_server("https://a.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}.png")
        
        else:
            self.widget.set_tile_server("https://a.basemaps.cartocdn.com/rastertiles/voyager/{z}/{x}/{y}.png")

    # Refresh
    def update(self, telemetry: dict, connection: dict) -> None:
        lat, lon = telemetry["lat"], telemetry["lon"]
        hlat, hlon, hset = connection["home_lat"], connection["home_lon"], connection["home_set"]

        home_known = hset and (hlat != 0.0 or hlon != 0.0)
        have_fix = lat != 0.0 or lon != 0.0

        if not have_fix and not self._map_initialised:
            if home_known and (hlat, hlon) != self._last_home:
                self.widget.set_position(hlat, hlon)
            elif not home_known and not self._map_fallback_done:
                self.widget.set_position(*FALLBACK_POS)
                self._map_fallback_done = True

        # V Marker
        if have_fix:
            heading = round(telemetry["heading"]) % 360

            if self._v_marker is None or heading != self._last_heading:
                icon = self._get_icon(heading)
                self._v_marker = self._replace_marker(self._v_marker, lat, lon, icon, "")
                self._last_heading = heading

            else:
                self._v_marker.set_position(lat, lon)
            
            if not self._map_initialised or self.follow_var.get():
                self.widget.set_position(lat, lon)
                self._map_initialised = True

        # Home Marker
        if home_known:
            if self._home_marker is None or (hlat, hlon) != self._last_home:
                self._home_marker = self._replace_marker(
                    self._home_marker, hlat, hlon, self._home_icon, "Home")
                self._last_home = (hlat, hlon)
        elif self._home_marker is not None:
            self._home_marker.delete()
            self._home_marker, self._last_home = None, (0.0, 0.0)

    def apply_theme(self, theme: dict) -> None:
        super().apply_theme(theme)

        self.frame.config(bg = theme["panel_bg"], fg = theme["fg"])

        self.toolbar.config(bg = theme["panel_bg"])
        self.chk_follow.config(bg = theme["panel_bg"], fg = theme["fg"],
                               activebackground = theme["panel_bg"], activeforeground = theme["fg"],
                               selectcolor = self.theme["checkbox"])