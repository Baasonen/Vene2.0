# Gamepad hw layer, mapping in control_mapping.py

from dataclasses import dataclass, field
from typing import List, Optional, Tuple

try:
    import pygame
    PYGAME_AVAIL = True
except ImportError:
    PYGAME_AVAIL = False

@dataclass
class RawGamepadState:
    axes: List[float] = field(default_factory = list)
    buttons: List[bool] = field(default_factory = list)
    hats: List[Tuple[int, int]] = field(default_factory = list)
    name: str = "None"

class GamepadInput:
    def __init__(self):
        self._joystick = None
        self._name = "None"

        if PYGAME_AVAIL:
            pygame.init()
            pygame.joystick.init()
            self._try_connect()

    def _try_connect(self) -> None:
        if pygame.joystick.get_count() == 0:
            return

        try:
            self._joystick = pygame.joystick.Joystick(0)
            self._joystick.init()
            self._name = self._joystick.get_name()

        except Exception:
            self._joystick = None
            self._name = "None"

    @property
    def connected(self) -> bool:
        return PYGAME_AVAIL and self._joystick is not None

    @property
    def name(self) -> str:
        return self._name

    def poll(self) -> Optional[RawGamepadState]:
        if not PYGAME_AVAIL:
            return None

        pygame.event.pump()

        if pygame.joystick.get_count() == 0:
            if self._joystick is not None:
                self._joystick = None
                self._name = "None"

            return None

        if self._joystick is None:
            self._try_connect()
            if self._joystick is None:
                return None

        try:
            axes = [self._joystick.get_axis(i) for i in range(self._joystick.get_numaxes())]
            buttons = [bool(self._joystick.get_button(i)) for i in range(self._joystick.get_numbuttons())]
            hats = [self._joystick.get_hat(i) for i in range(self._joystick.get_numhats())]

        except Exception:
            self._joystick = None
            self._name = "None"
            return None

        return RawGamepadState(axes = axes, buttons = buttons, hats = hats, name = self._name)