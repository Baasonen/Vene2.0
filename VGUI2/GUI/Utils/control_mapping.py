from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

from GUI.Utils.gamepad import RawGamepadState

@dataclass
class ControllerProfile:
    name: str

    rudder_axis: int
    rudder_max: float # Output scale (deg)
    rudder_deadzone: float = 0.1
    rudder_invert : bool = False

    # Throttle either form signle axis or both triggers

    throttle_axis: Optional[int] = None
    throttle_invert: bool = False
    throttle_deadzone: float = 0.1 # 0 ... 1

    throttle_trigger_r: Optional[int] = None
    throttle_trigger_l : Optional[int] = None
    trigger_deadzone: float = 0.05 # 0 ... 1

    throttle_max: float = 100.0 # Output scale (%)

    button_mode_map: Dict[int, int] = field(default_factory = dict)

    dpad_hat: Optional[int] = None # Hat idx
    course_step_deg: float = 5.0 # Deg per left / right press
    throttle_step_pct: float = 5.0 # % per up / down press

# Result of raw poll
@dataclass
class ControlInput:
    throttle: float
    rudder: float
    mode_requests: List[int]
    course_step: float = 0.0
    throttle_ap_step: float = 0.0

# Translate RawGamepadState -> ControlInput for specific ControllerProfile
class ControllerMapper:
    def __init__(self, profile: ControllerProfile):
        self.profile = profile
        self._button_states: Dict[int, bool] = {}
        self._hat_state: Tuple[int, int] = (0, 0)

    def reset(self) -> None:
        self._button_states.clear()
        self._hat_state = (0, 0)

    def map(self, raw: RawGamepadState) -> ControlInput:
        p = self.profile

        rudder = self._axis_value(raw, p.rudder_axis, p.rudder_deadzone)
        if p.rudder_invert:
            rudder *= -1.0
        rudder *= p.rudder_max

        if p.throttle_trigger_r is not None and p.throttle_trigger_l is not None:
            rt = self._trigger_value(raw, p.throttle_trigger_r, p.trigger_deadzone)
            lt = self._trigger_value(raw, p.throttle_trigger_l, p.trigger_deadzone)
            throttle = (rt - lt) * p.throttle_max

        elif p.throttle_axis is not None:
            throttle = self._axis_value(raw, p.throttle_axis, p.throttle_deadzone)
            if p.throttle_invert:
                throttle *= -1.0
            throttle *= p.throttle_max

        else:
            throttle = 0.0

        mode_requests = self._poll_mode_buttons(raw)
        course_step, throttle_ap_step = self._poll_dpad(raw)

        return ControlInput(throttle = throttle, rudder = rudder, mode_requests = mode_requests,
                            course_step = course_step, throttle_ap_step = throttle_ap_step)

    @staticmethod
    def _axis_value(raw: RawGamepadState, axis: int, deadzone: float) -> float:
        if axis >= len(raw.axes):
            return 0.0

        value = raw.axes[axis]
        return 0.0 if abs(value) < deadzone else value

    @staticmethod
    def _trigger_value(raw: RawGamepadState, axis: int, deadzone: float) -> float:
        if axis >= len(raw.axes):
            return 0.0

        normalized = max(0.0, (raw.axes[axis] + 1.0) / 2.0)
        return 0.0 if normalized < deadzone else normalized

    def _poll_dpad(self, raw: RawGamepadState) -> Tuple[float, float]:
        p = self.profile
        course_step = 0
        throttle_ap_step = 0.0

        if p.dpad_hat is None or p.dpad_hat >= len(raw.hats):
            self._hat_state = (0, 0)
            return course_step, throttle_ap_step

        x, y, = raw.hats[p.dpad_hat]
        prev_x, prev_y = self._hat_state

        if x != prev_x and x != 0:
            course_step = p.course_step_deg if x > 0 else -p.course_step_deg

        if y != prev_y and y != 0:
            throttle_ap_step = p.throttle_step_pct if y > 0 else -p.throttle_step_pct

        self._hat_state = (x, y)
        return course_step, throttle_ap_step

    def _poll_mode_buttons(self, raw: RawGamepadState) -> List[int]:
        requests: List[int] = []

        for button, mode in self.profile.button_mode_map.items():
            if button >= len(raw.buttons):
                continue

            pressed = raw.buttons[button]
            was_pressed = self._button_states.get(button, False)

            if pressed and not was_pressed:
                requests.append(mode)

            self._button_states[button] = pressed

        return requests

# Default xbox-style controller    
def make_xbox_profile() -> ControllerProfile:
    from VCOM.protocol import MODE_STOP, MODE_MANUAL, MODE_COURSE, MODE_AUTO
 
    BUTTON_A, BUTTON_B, BUTTON_X, BUTTON_Y = 0, 1, 2, 3
 
    return ControllerProfile(
        name = "Xbox",
        rudder_axis = 0,
        rudder_deadzone = 0.1,
        rudder_max = 80.0,
        throttle_trigger_r = 5,
        throttle_trigger_l = 4,
        trigger_deadzone = 0.05,
        throttle_max = 100.0,
        button_mode_map = {
            BUTTON_B: MODE_STOP,
            BUTTON_A: MODE_MANUAL,
            BUTTON_Y: MODE_AUTO,
            BUTTON_X: MODE_COURSE,
        },

        dpad_hat = 0,
        course_step_deg = 5.0,
        throttle_step_pct = 5.0,
    )

# Example profile for 2 sticks and no triggers
def make_simple_stick_profile() -> ControllerProfile:
    return ControllerProfile(
        name = "Simple dual-stick",
        rudder_axis = 3,
        rudder_deadzone = 0.05,
        rudder_max = 80.0,
        throttle_axis = 1,
        throttle_invert = True,
        throttle_deadzone = 0.05,
        throttle_max = 100.0,
        button_mode_map = {},
    )