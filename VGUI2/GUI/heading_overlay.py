import tkinter as tk
import math

from GUI.speedo_overlay import OVERLAY_X, OVERLAY_TOP_Y

COMPASS_SIZE = 92
COMPASS_PAD = 9
MAX_HEADING_ACC = 3
OVERLAY_GAP = 8

MAG_ACC_BIT_NAMES = ["HDG_A1", "HDG_A2", "HDG_A3"]

class HeadingOverlay:
    def __init__(self, parent, ctrl):
        self._parent = parent
        self._ctrl = ctrl

        self._heading_deg: float = 0.0
        self._have_heading: bool = False
        self._heading_acc: int = 0
        self._grid_color: str = "gray"
        self._pointer_color: str = "red"

        self._acc_bits = [self._find_bit(name) for name in MAG_ACC_BIT_NAMES]

        self.frame = tk.Frame(parent, bd = 1, relief = "solid")
        self.frame.place(in_ = parent, relx = 1.0, rely = 0.0, x = OVERLAY_X, y = OVERLAY_TOP_Y, anchor = "ne")

        self.lbl_heading_caption = tk.Label(self.frame, text = "HEADING", font = ("Segoe UI", 8, "bold"))
        self.lbl_heading_caption.pack(padx = 10, pady = (6, 0))

        self.lbl_heading_val = tk.Label(self.frame, text = "---°", font = ("Segoe UI", 18, "bold"))
        self.lbl_heading_val.pack(padx = 10)

        self.canvas = tk.Canvas(self.frame, width = COMPASS_SIZE, height = COMPASS_SIZE, highlightthickness = 0)
        self.canvas.pack(padx = 10, pady = (2, 4))

        self.lbl_acc_val = tk.Label(self.frame, text = f"Acc: 0 / {MAX_HEADING_ACC}", font = ("Segoe UI", 11))
        self.lbl_acc_val.pack(padx = 10, pady = (0, 8))

        self.spacer = tk.Frame(self.frame, height = 0, bd = 0, highlightthickness = 0)
        self.spacer.pack()

        self._draw_compass()
        self.frame.lift()

    def _find_bit(self, name: str):
        for bit, (defname, _desc) in self._ctrl.error_defs.items():
            if defname == name:
                return bit

        return None

    def _draw_compass(self) -> None:
        self.canvas.delete("all")

        c = COMPASS_SIZE / 2
        r = c - COMPASS_PAD

        for i in range(8):
            ang = math.radians(i * 45)
            x1, y1 = c + r * 0.82 * math.sin(ang), c - r * 0.82 * math.cos(ang)
            x2, y2 = c + r * math.sin(ang), c - r * math.cos(ang)
            self.canvas.create_line(x1, y1, x2, y2, fill = self._grid_color, tags = "grid")

        for i in range(36):
            ang = math.radians(i * 10)
            x1, y1 = c + r * 0.92 * math.sin(ang), c - r * 0.92 * math.cos(ang)
            x2, y2 = c + r * math.sin(ang), c - r * math.cos(ang)
            self.canvas.create_line(x1, y1, x2, y2, fill = self._grid_color, tags = "grid")

        self.canvas.create_oval(c - r, c - r, c + r, c + r, outline = self._grid_color, tags = "grid")

        self._draw_pointer()

    def _draw_pointer(self) -> None:
        self.canvas.delete("pointer")

        c = COMPASS_SIZE / 2
        r = c - COMPASS_PAD

        if not self._have_heading:
            self.canvas.create_oval(c - 4, c - 4, c + 4, c + 4, fill = self._pointer_color,
                                    outline = "", tags = "pointer")
            return

        ang = math.radians(self._heading_deg)
        tip_x, tip_y = c + r * 0.8 * math.sin(ang), c - r * 0.8 * math.cos(ang)

        back1 = math.radians(self._heading_deg + 150)
        back2 = math.radians(self._heading_deg - 150)
        bx1, by1 = c + r * 0.25 * math.sin(back1), c - r * 0.25 * math.cos(back1)
        bx2, by2 = c + r * 0.25 * math.sin(back2), c - r * 0.25 * math.cos(back2)

        self.canvas.create_polygon(tip_x, tip_y, bx1, by1, bx2, by2,
                                   fill = self._pointer_color, outline = "", tags = "pointer")
        self.canvas.create_oval(c - 3, c - 3, c + 3, c + 3, fill = self._pointer_color,
                                outline = "", tags = "pointer")

    def update(self, telemetry: dict) -> None:
        error = telemetry.get("error", 0)

        level = 0
        for i, bit in enumerate(self._acc_bits, start = 1):
            if bit is not None and error >> bit & 1:
                level = i

        self._heading_acc = level
        self._have_heading = level > 0
        self._heading_deg = telemetry["heading"] % 360 if self._have_heading else 0.0
        
        self.lbl_heading_val.config(text = f"{round(self._heading_deg):03d}°" if self._have_heading else "---°")
        self.lbl_acc_val.config(text = f"Acc: {self._heading_acc} / {MAX_HEADING_ACC}")

        self._draw_pointer()

    def set_visible(self, visible: bool, y: int = OVERLAY_TOP_Y) -> None:
        if visible:
            self.frame.place(in_ = self._parent, relx = 1.0, rely = 0.0, x = OVERLAY_X, y = y, anchor = "ne")
        else:
            self.frame.place_forget()

    def set_min_width(self, width: int) -> None:
        self.spacer.config(width = width)

    def apply_theme(self, theme: dict) -> None:
        self._grid_color = theme["fg_dim"]
        self._pointer_color = theme["red"]

        self.frame.config(bg = theme["panel_bg"], highlightbackground = theme["border"],
                          highlightthickness = 1)
        self.lbl_heading_caption.config(bg = theme["panel_bg"], fg = theme["fg_dim"])
        self.lbl_heading_val.config(bg = theme["panel_bg"], fg = theme["fg"])
        self.lbl_acc_val.config(bg = theme["panel_bg"], fg = theme["fg_dim"])
        self.canvas.config(bg = theme["panel_bg"])
        self.spacer.config(bg = theme["panel_bg"])

        self._draw_compass()