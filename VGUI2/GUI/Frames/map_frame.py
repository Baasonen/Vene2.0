import tkinter as tk
from tkinter import ttk, filedialog
from typing import Callable, Tuple, Optional
from PIL import Image, ImageDraw, ImageTk
import tkintermapview
from tkintermapview import map_widget as _tmv_map_widget
import requests
import os
import math

from GUI.base_frame import BaseFrame
from GUI.speedo_overlay import SpeedoOverlay, OVERLAY_TOP_Y
from GUI.heading_overlay import HeadingOverlay, OVERLAY_GAP
from VCOM.protocol import MODE_COURSE

# Fix for slow OpenSeaMap
class _TimeoutRequests:
    def __getattr__(self, name):
        return getattr(requests, name)

    @staticmethod
    def get(*args, **kwargs):
        kwargs.setdefault("timeout", 5)
        return requests.get(*args, **kwargs)

_tmv_map_widget.requests = _TimeoutRequests()

# Required for tkinterMapView overlays
if not hasattr(Image, "ANTIALIAS"):
    Image.ANTIALIAS = Image.LANCZOS

ICON_SIZE = 30
HOME_ICON_SIZE = 20
FALLBACK_POS = (60.1849, 24.8250)

ROUTES_DIR = os.path.join(os.getcwd(), "Routes")

BASEMAP_OPTIONS = ["Default", "Satellite", "OpenStreetMap"]
OVERLAY_OPTIONS = ["None", "Speed", "Heading", "Both"]

BASE_TILE_SERVERS = {
    "Default": {
        False: "https://a.basemaps.cartocdn.com/rastertiles/voyager/{z}/{x}/{y}.png",
        True: "https://a.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}.png",
    },
    "Satellite": "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
    "OpenStreetMap": "https://a.tile.openstreetmap.org/{z}/{x}/{y}.png",
}

SEAMARK_OVERLAY = "http://tiles.openseamap.org/seamark//{z}/{x}/{y}.png"

base_path = os.path.join(os.path.dirname(__file__), "..", "..")

class MapFrame(BaseFrame):
    def __init__(self, parent, theme, ctrl,
                 on_add_waypoint: Callable[[Tuple[float, float]], None],
                 on_set_home: Callable[[Tuple[float, float]], None],
                 on_save_route: Callable[[str], None],
                 on_load_route: Callable[[str], None],
                 on_pattern_planner: Callable[[], None] = None):
        
        self.on_add_waypoint = on_add_waypoint
        self.on_set_home = on_set_home
        self.on_save_route = on_save_route
        self.on_load_route = on_load_route
        self.on_pattern_planner = on_pattern_planner

        self._point_pick_callback = None

        self._v_marker = None
        self._v_icon_cache: dict[int, ImageTk.PhotoImage] = {}
        self._home_marker = None
        self._home_icon = None
        self._map_initialised = False
        self._map_fallback_done = False
        self._last_heading = -1
        self._last_home = (0.0, 0.0)
        self._is_dark = False

        try:
            home_icon_path = os.path.join(base_path, "Icons", "green_dot.png")
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

        self.follow_var = tk.BooleanVar(value = False)
        self.chk_follow = tk.Checkbutton(
            self.toolbar, text = "Center on target", variable = self.follow_var,
            font = ("Segoe UI", 9), bg = self.theme["panel_bg"], fg = self.theme["fg"],
            activebackground = self.theme["panel_bg"], activeforeground = self.theme["fg"],
            selectcolor = self.theme["checkbox"],
        )
        self.chk_follow.pack(side = "left", padx = 5)

        ttk.Button(self.toolbar, text = "Snap to target", command = self._center_on_target).pack(side = "left", padx = 6)

        ttk.Button(self.toolbar, text = "Save Route", command = self._on_save_route_click).pack(side = "left", padx = 6)
        ttk.Button(self.toolbar, text = "Load Route", command = self._on_load_route_click).pack(side = "left", padx = 6)

        ttk.Button(self.toolbar, text = "Pattern Planner", command = self._on_pattern_planner_click).pack(side = "left", padx = 6)

        self.basemap_var = tk.StringVar(value = "Satellite")
        self.cmb_basemap = ttk.Combobox(
            self.toolbar, textvariable = self.basemap_var, values = BASEMAP_OPTIONS,
            state = "readonly", width = 12,
        )
        self.cmb_basemap.bind("<<ComboboxSelected>>", lambda _event: self._refresh_base_tiles())
        self.cmb_basemap.pack(side = "right", padx = 6)

        self.seamark_var = tk.BooleanVar(value = False)
        self.chk_seamark = tk.Checkbutton(
            self.toolbar, text = "Charts", variable = self.seamark_var,
            font = ("Segoe UI", 12), bg = self.theme["panel_bg"], fg = self.theme["fg"],
            activebackground = self.theme["panel_bg"], activeforeground = self.theme["fg"],
            selectcolor = self.theme["checkbox"], command = self._refresh_overlay,
        )
        self.chk_seamark.pack(side = "right", padx = 5)

        self.overlay_var = tk.StringVar(value = "Both")
        self.cmb_overlay = ttk.Combobox(
            self.toolbar, textvariable = self.overlay_var, values = OVERLAY_OPTIONS,
            state = "readonly", width = 8,
        )
        self.cmb_overlay.bind("<<ComboboxSelected>>", lambda _event: self._on_overlay_mode_change())
        self.cmb_overlay.pack(side = "right", padx = 6)

        tk.Label(self.toolbar, text = "Nav Overlay: ", font = ("Segoe UI", 9), bg = self.theme["panel_bg"], fg = self.theme["fg"]
                        ).pack(side = "right", padx = (6, 2))

        self.widget = tkintermapview.TkinterMapView(self.frame, corner_radius = 4)
        self.widget.pack(fill = "both", expand = True)
        self.widget.set_zoom(16)

        self.widget.add_right_click_menu_command(
            label = "Add Waypoint", command = self.on_add_waypoint, pass_coords = True
        )

        self.widget.add_right_click_menu_command(
            label = "Set Home WP", command = self.on_set_home, pass_coords = True
        )

        self.widget.canvas.bind("<Double-Button-1>", self._on_map_double_click)

        self.speedo = SpeedoOverlay(self.widget)
        self.heading = HeadingOverlay(self.widget)

        self.speedo.frame.bind("<Configure>", self._sync_overlays)

        self._on_overlay_mode_change()

        self._v_img_orig = self._load_icon()
        if self._home_icon == None:
            self._home_icon = ImageTk.PhotoImage(self._make_home_pin_icon())

        self.apply_theme(self.theme)

    def _sync_overlays(self, event = None) -> None:
        if self.overlay_var.get() == "Both":
            width = event.width if event else self.speedo.frame.winfo_reqwidth()
            height = event.height if event else self.speedo.frame.winfo_reqheight()
            
            self.heading.set_min_width(width - 4)
            self.heading.set_visible(True, y = OVERLAY_TOP_Y + height + OVERLAY_GAP)

    def _on_overlay_mode_change(self) -> None:
        mode = self.overlay_var.get()

        show_speedo = mode in ("Speed", "Both")
        show_heading = mode in ("Heading", "Both")

        self.speedo.set_visible(show_speedo)

        if show_heading:
            if show_speedo:
                self.widget.update_idletasks()
                y = OVERLAY_TOP_Y + self.speedo.frame.winfo_reqheight() + OVERLAY_GAP

                self._sync_overlays()
            else:
                y = OVERLAY_TOP_Y

                self.heading.set_min_width(0)

            self.heading.set_visible(True, y = y)
        else:
            self.heading.set_visible(False)

    # Icon 
    @staticmethod
    def _load_icon() -> Image.Image:
        try:
            raw = Image.open("Icons/vene1.png").convert("RGBA")
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
    
    def _replace_marker(self, old_marker, lat, lon, icon, text):
        if old_marker is not None:
            old_marker.delete()

        return self.widget.set_marker(lat, lon, icon = icon, text = text, text_color = self.theme["red"])
    
    def _center_on_target(self) -> None:
        d = self.ctrl.get_telemetry_data()
        if d["lat"] or d["lon"]:
            self.widget.set_position(d["lat"], d["lon"])

    def _on_save_route_click(self) -> None:
        os.makedirs(ROUTES_DIR, exist_ok = True)

        path = filedialog.asksaveasfilename(
            defaultextension = ".rte",
            filetypes = [("Route", "*.rte"), ("All files", "*.*")],
            title = "Save Route",
            initialdir = ROUTES_DIR,
        )

        if path:
            self.on_save_route(path)

    def _on_load_route_click(self) -> None:
        os.makedirs(ROUTES_DIR, exist_ok = True)

        path = filedialog.askopenfilename(
            defaultextension = ".rte",
            filetypes = [("Route", "*.rte"), ("All files", "*.*")],
            title = "Load Route",
            initialdir = ROUTES_DIR,
        )

        if path:
            points = self.on_load_route(path)
            self._focus_on_points(points)

    def _focus_on_points(self, points) -> None:
        if not points:
            return

        lats = [p[0] for p in points]
        lons = [p[1] for p in points]

        if len(points) == 1:
            self.widget.set_position(lats[0], lons[0])
            return

        try:
            self.widget.fit_bounding_box((max(lats), min(lons)), (min(lats), max(lons)))
        except Exception:
            self.widget.set_position(sum(lats) / len(lats), sum(lons) / len(lons))

    def _on_pattern_planner_click(self) -> None:
        if self.on_pattern_planner is not None:
            self.on_pattern_planner()

    # Arms a single shot map click, route the next map double clik to callback, pass None to cancel pending pick
    def request_point(self, callback: Optional[Callable[[Tuple[float, float]], None]]) -> None:
        self._point_pick_callback = callback

    def _on_map_double_click(self, event) -> None:
        lat, lon = self.widget.convert_canvas_coords_to_decimal_coords(event.x, event.y)

        if self._point_pick_callback is not None:
            callback = self._point_pick_callback
            self._point_pick_callback = None
            callback((lat, lon))
            return

        telemetry = self.ctrl.get_telemetry_data()

        if telemetry.get("mode") == MODE_COURSE:
            vlat, vlon = telemetry["lat"], telemetry["lon"]

            if vlat != 0.0 and vlon != 0.0 and telemetry.get("hAcc") < 10.0:
                self.ctrl.set_course(self._bearing_to(vlat, vlon, lat, lon))

            return

        self.on_add_waypoint((lat, lon))

    @staticmethod
    def _bearing_to(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        dlambda = math.radians(lon2 - lon1)

        x = math.sin(dlambda) * math.cos(phi2)
        y = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlambda)

        return (math.degrees(math.atan2(x, y)) + 360) % 360

    # Tile Server
    def set_tiles(self, is_dark: bool) -> None:
        self._is_dark = is_dark
        self.widget.canvas.config(bg = self.theme["canvas_bg"])
        self._refresh_base_tiles()

    def _refresh_base_tiles(self) -> None:
        server = self._resolve_base_tile_server()
        if server != self.widget.tile_server:
            self.widget.set_tile_server(server)

    def _resolve_base_tile_server(self) -> str:
        server = BASE_TILE_SERVERS[self.basemap_var.get()]
        return server[self._is_dark] if isinstance(server, dict) else server

    def _refresh_overlay(self) -> None:
        self.widget.set_overlay_tile_server(SEAMARK_OVERLAY if self.seamark_var.get() else None)

        # set_overlay_tile_server() doesnt clear tile cache, reissue set_tile_server() to clear

        self.widget.set_tile_server(self.widget.tile_server)

    # Refresh
    def update(self, telemetry: dict, connection: dict) -> None:
        self.speedo.update(telemetry)
        self.heading.update(telemetry)

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
        self.chk_seamark.config(bg = theme["panel_bg"], fg = theme["fg"],
                                activebackground = theme["panel_bg"], activeforeground = theme["fg"],
                                selectcolor = self.theme["checkbox"])

        self.speedo.apply_theme(theme)
        self.heading.apply_theme(theme)