import tkinter as tk
from tkinter import ttk
from typing import Dict, Optional, Tuple

from GUI.base_frame import BaseFrame
from vcom.protocol import MODE_MANUAL, MODE_COURSE, MODE_AUTO, MODE_RTH

try:
    import pygame
    PYGAME_AVAIL = True
except ImportError:
    PYGAME_AVAIL = False

STICK_DZ = 0.1
TRIGGER_DZ = 0.05

AXIS_RUDDER = 0
AXIS_THROTTLE_RT = 5
AXIS_THROTTLE_LT = 4

POLL_MS = 50

THROTTLE_MAX = 100.0
RUDDER_MAX = 80.0

THROTTLE_RAMP_RATE = 100.0
RUDDER_RAMP_RATE = 200.0

class ManualControlFrame(BaseFrame):
    def __init__(self, parent, theme, ctrl, root: tk.Tk):
        self.root = root
        
        self._keys_down: Dict[str, bool] = {
            "up": False,
            "down": False,
            "left": False,
            "right": False,
            "w": False,
            "a": False,
            "s": False,
            "d": False,
        }

        self._joystick = None
        self._gamepad_name = "None"

        self._current_mode = 0
        self._throttle = 0.0
        self._rudder = 0.0

        super().__init__(parent, theme, ctrl)

        self._bind_keys()
        self._init_gamepad()
        self._poll()

    def build(self) -> None:
        self.frame = tk.Frame(self.parent, bg = self.theme["panel_bg"], bd = 1,
                               relief = "solid", highlightbackground = self.theme["border"])
        self.frame.pack(fill = "x", side = "top", pady = (0, 6))

        self.header_row = tk.Frame(self.frame, bg = self.theme["panel_bg"])
        self.header_row.pack(fill = "x", padx = 8, pady = (6, 4))

        self.lbl_title = tk.Label(self.header_row, text = "MANUAL CONTROL",
                                   font = ("Segoe UI", 9, "bold"),
                                   bg = self.theme["panel_bg"], fg = self.theme["fg"])
        self.lbl_title.pack(side = "left")

        self.lbl_gamepad = tk.Label(self.header_row, text = "Keyboard Control",
                                     font = ("Segoe UI", 8),
                                     bg = self.theme["panel_bg"], fg = self.theme["fg_dim"])
        self.lbl_gamepad.pack(side = "right")

        self.thr_row = tk.Frame(self.frame, bg = self.theme["panel_bg"])
        self.thr_row.pack(fill = "x", padx = 8, pady = (0, 3))

        self.lbl_throttle_name = tk.Label(self.thr_row, text = "Throttle", font = ("Segoe UI", 8),
                                           width = 7, anchor = "w",
                                           bg = self.theme["panel_bg"], fg = self.theme["fg_dim"])
        self.lbl_throttle_name.pack(side = "left")

        self.throttle_canvas = tk.Canvas(self.thr_row, height = 14, bg = self.theme["canvas_bg"],
                                          highlightthickness = 0)
        self.throttle_canvas.pack(side = "left", fill = "x", expand = True, padx = (4, 6))
        self.throttle_canvas.bind("<Configure>", lambda e: self._draw_bars())

        self.lbl_throttle_val = tk.Label(self.thr_row, text = "0", font = ("Segoe UI", 8),
                                          width = 4, anchor = "e",
                                          bg = self.theme["panel_bg"], fg = self.theme["fg"])
        self.lbl_throttle_val.pack(side = "left")

        self.rud_row = tk.Frame(self.frame, bg = self.theme["panel_bg"])
        self.rud_row.pack(fill = "x", padx = 8, pady = (0, 6))

        self.lbl_rudder_name = tk.Label(self.rud_row, text = "Rudder", font = ("Segoe UI", 8),
                                         width = 7, anchor = "w",
                                         bg = self.theme["panel_bg"], fg = self.theme["fg_dim"])
        self.lbl_rudder_name.pack(side = "left")

        self.rudder_canvas = tk.Canvas(self.rud_row, height = 14, bg = self.theme["canvas_bg"],
                                        highlightthickness = 0)
        self.rudder_canvas.pack(side = "left", fill = "x", expand = True, padx = (4, 6))
        self.rudder_canvas.bind("<Configure>", lambda e: self._draw_bars())

        self.lbl_rudder_val = tk.Label(self.rud_row, text = "0", font = ("Segoe UI", 8),
                                        width = 4, anchor = "e",
                                        bg = self.theme["panel_bg"], fg = self.theme["fg"])
        self.lbl_rudder_val.pack(side = "left")

        self._draw_bars()

        # Course
        self.course_frame = tk.Frame(self.frame, bg = self.theme["panel_bg"], bd = 1,
                                      relief = "solid", highlightbackground = self.theme["border"])
        self.course_frame.pack(fill = "x", padx = 8, pady = (0, 8))

        self.course_header_row = tk.Frame(self.course_frame, bg = self.theme["panel_bg"])
        self.course_header_row.pack(fill = "x", padx = 6, pady = (5, 3))

        self.lbl_course_title = tk.Label(self.course_header_row, text = "COURSE",
                                          font = ("Segoe UI", 8, "bold"),
                                          bg = self.theme["panel_bg"], fg = self.theme["fg_dim"])
        self.lbl_course_title.pack(side = "left")

        self.course_input_row = tk.Frame(self.course_frame, bg = self.theme["panel_bg"])
        self.course_input_row.pack(fill = "x", padx = 6, pady = (0, 6))

        self.course_entry = tk.Entry(self.course_input_row, width = 6, font = ("Segoe UI", 12),
                                      bg = self.theme["canvas_bg"], fg = self.theme["fg"],
                                      insertbackground = self.theme["fg"], relief = "flat")
        self.course_entry.pack(side = "left", padx = (0, 6))
        self.course_entry.bind("<Return>", lambda e: self._on_set_course())

        self.btn_set_course = ttk.Button(self.course_input_row, text = "SET",
                                  command = self._on_set_course)
        self.btn_set_course.pack(side = "left", padx = (0, 12))

        self.lbl_course_readout = tk.Label(self.course_input_row, text = "--.-°",
                                            font = ("Consolas", 12, "bold"),
                                            bg = self.theme["panel_bg"], fg = self.theme["fg_dim"])
        self.lbl_course_readout.pack(side = "left")

        # A/P Throttle
        self.thr_ap_frame = tk.Frame(self.frame, bg = self.theme["panel_bg"], bd = 1,
                                      relief = "solid", highlightbackground = self.theme["border"])
        self.thr_ap_frame.pack(fill = "x", padx = 8, pady = (0, 8))

        self.thr_ap_header_row = tk.Frame(self.thr_ap_frame, bg = self.theme["panel_bg"])
        self.thr_ap_header_row.pack(fill = "x", padx = 6, pady = (5, 3))

        self.lbl_thr_ap_title = tk.Label(self.thr_ap_header_row, text = "A/P THROTTLE",
                                          font = ("Segoe UI", 8, "bold"),
                                          bg = self.theme["panel_bg"], fg = self.theme["fg_dim"])
        self.lbl_thr_ap_title.pack(side = "left")

        self.thr_ap_input_row = tk.Frame(self.thr_ap_frame, bg = self.theme["panel_bg"])
        self.thr_ap_input_row.pack(fill = "x", padx = 6, pady = (0, 6))

        self.thr_ap_entry = tk.Entry(self.thr_ap_input_row, width = 6, font = ("Segoe UI", 12),
                                      bg = self.theme["canvas_bg"], fg = self.theme["fg"],
                                      insertbackground = self.theme["fg"], relief = "flat")
        self.thr_ap_entry.pack(side = "left", padx = (0, 6))
        self.thr_ap_entry.bind("<Return>", lambda e: self._on_set_throttle_ap())

        self.btn_set_thr_ap = ttk.Button(self.thr_ap_input_row, text = "SET",
                                  command = self._on_set_throttle_ap)
        self.btn_set_thr_ap.pack(side = "left", padx = (0, 12))

        self.lbl_thr_ap_readout = tk.Label(self.thr_ap_input_row, text = "--%",
                                            font = ("Consolas", 12, "bold"),
                                            bg = self.theme["panel_bg"], fg = self.theme["fg_dim"])
        self.lbl_thr_ap_readout.pack(side = "left")

    def _bind_keys(self) -> None:
        bindings = {
            "<KeyPress-Up>": ("up", True), "<KeyRelease-Up>": ("up", False),
            "<KeyPress-Down>": ("down", True), "<KeyRelease-Down>": ("down", False),
            "<KeyPress-Left>": ("left", True), "<KeyRelease-Left>": ("left", False),
            "<KeyPress-Right>": ("right", True), "<KeyRelease-Right>": ("right", False),
            "<KeyPress-w>": ("w", True), "<KeyRelease-w>": ("w", False),
            "<KeyPress-W>": ("w", True), "<KeyRelease-W>": ("w", False),
            "<KeyPress-a>": ("a", True), "<KeyRelease-a>": ("a", False),
            "<KeyPress-A>": ("a", True), "<KeyRelease-A>": ("a", False),
            "<KeyPress-s>": ("s", True), "<KeyRelease-s>": ("s", False),
            "<KeyPress-S>": ("s", True), "<KeyRelease-S>": ("s", False),
            "<KeyPress-d>": ("d", True), "<KeyRelease-d>": ("d", False),
            "<KeyPress-D>": ("d", True), "<KeyRelease-D>": ("d", False),
        }

        for seq, (key, state) in bindings.items():
            self.root.bind_all(seq, lambda e, k = key, s = state: self._set_key(k, s))

    def _set_key(self, key: str, state: bool) -> None:
        self._keys_down[key] = state

    def _keyboard_inputs(self) -> Tuple[float, float]:
        throttle = 0.0
        rudder = 0.0

        if self._keys_down["up"] or self._keys_down["w"]:
            throttle += THROTTLE_MAX

        if self._keys_down["down"] or self._keys_down["s"]:
            throttle -= THROTTLE_MAX

        if self._keys_down["right"] or self._keys_down["d"]:
            rudder += RUDDER_MAX

        if self._keys_down["left"] or self._keys_down["a"]:
            rudder -= RUDDER_MAX

        return throttle, rudder
    
    def _ramped_keyboard_inputs(self) -> Tuple[float, float]:
        target_throttle, target_rudder = self._keyboard_inputs()

        throttle_step = THROTTLE_RAMP_RATE * (POLL_MS / 1000.0)
        rudder_step = RUDDER_RAMP_RATE * (POLL_MS / 1000.0)

        throttle = self._step_toward(self._throttle, target_throttle, throttle_step)
        rudder = self._step_toward(self._rudder, target_rudder, rudder_step)
        
        return throttle, rudder
    
    def _on_set_course(self) -> None:
        text = self.course_entry.get().strip()

        try:
            course = float(text)
        except ValueError:
            self.course_entry.config(bg = self.theme["red"])
            self.root.after(400, lambda: self.course_entry.config(bg = self.theme["canvas_bg"]))
            return
        
        course = course % 360
        self.ctrl.set_course(course)
        self.course_entry.delete(0, tk.END)

    def _on_set_throttle_ap(self) -> None:
        text = self.thr_ap_entry.get().strip()

        try:
            throttle = float(text)
        except ValueError:
            self.thr_ap_entry.config(bg = self.theme["red"])
            self.root.after(400, lambda: self.thr_ap_entry.config(bg = self.theme["canvas_bg"]))
            return

        throttle = max(-100, min(100, throttle))
        self.ctrl.set_throttle(throttle)
        self.thr_ap_entry.delete(0, tk.END)


    @staticmethod
    def _step_toward(current: float, target: float, max_step: float) -> float:
        if current < target:
            return min(current + max_step, target)
        
        if current > target:
            return max(current - max_step, target)
        
        return current
    
    def _init_gamepad(self) -> None:
        if not PYGAME_AVAIL:
            self.lbl_gamepad.config(text = "Gamepad: pygame not available")
            return
        
        try: 
            pygame.init()
            pygame.joystick.init()
        except Exception as e:
            print(f"[MANUAL] Pygame init failed: {e}")

    def _refresh_gamepad(self) -> None:
        if not PYGAME_AVAIL:
            return
        
        if pygame.joystick.get_count() == 0:
            if self._joystick is not None:
                self._joystick = None
                self._gamepad_name = "None"

            return
                

        if self._joystick is None:
            try:
                self._joystick = pygame.joystick.Joystick(0)
                self._joystick.init()
                self._gamepad_name = self._joystick.get_name()
            except Exception:
                self._joystick = None
                self._gamepad_name = "None"

    def _gamepad_input(self) -> Optional[Tuple[float, float]]:
        if self._joystick is None:
            return None
        
        try:
            pygame.event.pump()

            rudder_axis = self._joystick.get_axis(AXIS_RUDDER)

            if abs(rudder_axis) < STICK_DZ:
                rudder_axis = 0.0

            n_axes = self._joystick.get_numaxes()
            rt = self._joystick.get_axis(AXIS_THROTTLE_RT) if n_axes > AXIS_THROTTLE_RT else -1.0
            lt = self._joystick.get_axis(AXIS_THROTTLE_LT) if n_axes > AXIS_THROTTLE_LT else -1.0

            rt_n = max(0.0, (rt + 1.0) / 2.0)
            lt_n = max(0.0, (lt + 1.0) / 2.0)

            if rt_n < TRIGGER_DZ:
                rt_n = 0.0

            if lt_n < TRIGGER_DZ:
                lt_n = 0.0

            throttle = (rt_n - lt_n) * THROTTLE_MAX
            rudder = rudder_axis * RUDDER_MAX

            return throttle, rudder
        
        except Exception:
            self._joystick = None
            self._gamepad_name = "None"
            
            return None
        
    def _poll(self) -> None:
        self._refresh_gamepad()

        if self._joystick is not None:
            result = self._gamepad_input()
            throttle, rudder = result if result is not None else (0.0, 0.0)

        else:
            throttle, rudder = self._ramped_keyboard_inputs()

        if self._current_mode == MODE_MANUAL:
            self._throttle = throttle
            self._rudder = rudder

        else:
            self._throttle = 0.0
            self._rudder = 0.0

        self.ctrl.manual_throttle = self._throttle
        self.ctrl.manual_rudder = self._rudder

        self._draw_bars()

        gp_text = (f"{self._gamepad_name}" if self._joystick else "Keyboard Control")
        self.lbl_gamepad.config(text = gp_text)

        course, valid = self.ctrl.get_course_status()

        if valid:
            text = f"{course:.1f}°"

            if self._current_mode == MODE_COURSE:
                color = self.theme["green"]

            else:
                color = self.theme["fg"]

            self.lbl_course_readout.config(text = text, fg = color)

        else:
            self.lbl_course_readout.config(text = "--.-°", fg = self.theme["fg_dim"])

        throttle_ap, thr_valid = self.ctrl.get_throttle_status()

        if thr_valid:
            text = f"{throttle_ap:.0f}%"

            if self._current_mode in (MODE_AUTO, MODE_RTH, MODE_COURSE):
                color = self.theme["green"]

            else:
                color = self.theme["fg"]

            self.lbl_thr_ap_readout.config(text = text, fg = color)

        else:
            self.lbl_thr_ap_readout.config(text = "--%", fg = self.theme["fg_dim"])

        self.root.after(POLL_MS, self._poll)

    def _draw_bars(self) -> None:
        if not hasattr(self, "throttle_canvas"):
            return

        self._draw_hbar(self.throttle_canvas, self._throttle, THROTTLE_MAX, reverse_color = True)
        self.lbl_throttle_val.config(text = f"{self._throttle:.0f}")

        self._draw_hbar(self.rudder_canvas, self._rudder, RUDDER_MAX, reverse_color = False)
        self.lbl_rudder_val.config(text = f"{self._rudder:.0f}")

    def _draw_hbar(self, canvas: tk.Canvas, value: float, max_value: float, reverse_color: bool = False) -> None:
        canvas.delete("all")
        w = canvas.winfo_width()
        if w < 10:
            w = 160

        h = int(canvas["height"])
        mid = w / 2
        frac = value / max_value
        bar_w = abs(frac) * mid

        x0 = mid - bar_w if frac < 0 else mid
        x1 = mid if frac < 0 else mid + bar_w

        fill = self.theme["red"] if (reverse_color and frac < 0) else self.theme["accent"]

        canvas.create_line(mid, 0, mid, h, fill = self.theme["fg_dim"])
        canvas.create_rectangle(x0, 2, x1, h - 2, fill = fill, width = 0)

    def apply_theme(self, theme: Dict) -> None:
        super().apply_theme(theme)

        if not hasattr(self, "lbl_title"):
            return

        self.frame.config(bg = theme["panel_bg"], highlightbackground = theme["border"])
        self.header_row.config(bg = theme["panel_bg"])
        self.thr_row.config(bg = theme["panel_bg"])
        self.rud_row.config(bg = theme["panel_bg"])

        self.lbl_title.config(bg = theme["panel_bg"], fg = theme["fg"])
        self.lbl_gamepad.config(bg = theme["panel_bg"], fg = theme["fg_dim"])
        self.throttle_canvas.config(bg = theme["canvas_bg"])
        self.rudder_canvas.config(bg = theme["canvas_bg"])
        self.lbl_throttle_val.config(bg = theme["panel_bg"], fg = theme["fg"])
        self.lbl_rudder_val.config(bg = theme["panel_bg"], fg = theme["fg"])

        for child in self.throttle_canvas.master.winfo_children():
            if isinstance(child, tk.Label):
                child.config(bg = theme["panel_bg"], fg = theme["fg_dim"])
        for child in self.rudder_canvas.master.winfo_children():
            if isinstance(child, tk.Label):
                child.config(bg = theme["panel_bg"], fg = theme["fg_dim"])

        self.course_frame.config(bg = theme["panel_bg"], highlightbackground = theme["border"])
        self.course_header_row.config(bg = theme["panel_bg"])
        self.course_input_row.config(bg = theme["panel_bg"])

        self.lbl_course_title.config(bg = theme["panel_bg"], fg = theme["fg_dim"])
        self.course_entry.config(bg = theme["canvas_bg"], fg = theme["fg"], insertbackground = theme["fg"])

        self.lbl_course_readout.config(bg = theme["panel_bg"])

        self.thr_ap_frame.config(bg = theme["panel_bg"], highlightbackground = theme["border"])
        self.thr_ap_header_row.config(bg = theme["panel_bg"])
        self.thr_ap_input_row.config(bg = theme["panel_bg"])

        self.lbl_thr_ap_title.config(bg = theme["panel_bg"], fg = theme["fg_dim"])
        self.thr_ap_entry.config(bg = theme["canvas_bg"], fg = theme["fg"], insertbackground = theme["fg"])

        self.lbl_thr_ap_readout.config(bg = theme["panel_bg"])

        self._draw_bars()
 
    def update(self, telemetry: dict, connection: dict) -> None:
        self._current_mode = telemetry.get("mode", 0)