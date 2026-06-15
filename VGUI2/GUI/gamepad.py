
try:
    import pygame
    PYGAME_AVAIL = True
except ImportError:
    PYGAME_AVAIL = False

class GamepadInput:
    THROTTLE_DEADZONE = 5
    RUDDER_DEADZONE = 4

    def __init__(self):
        self.joystick = None

        if PYGAME_AVAIL:
            pygame.init()
            pygame.joystick.init()
            self._try_connect()

    def _try_connect(self):
        if pygame.joystick.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()

    @property
    def connected(self) -> bool:
        return PYGAME_AVAIL and self.joystick is not None
    
    def poll(self):
        if not PYGAME_AVAIL:
            return None
        
        pygame.event.pump()

        if pygame.joystick.get_count() == 0:
            self.joystick = None
            return None
        
        if self.joystick is None:
            self._try_connect()
            if self.joystick is None:
                return None
            
        throttle = -self.joystick.get_axis(1) * 100
        rudder = self.joystick.get_axis(3) * 80

        if abs(throttle) < self.THROTTLE_DEADZONE:
            throttle = 0

        if abs(rudder) < self.RUDDER_DEADZONE:
            rudder = 0

        return throttle, rudder, self.joystick.get_name()