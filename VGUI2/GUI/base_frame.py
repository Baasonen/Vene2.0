class BaseFrame:
    def __init__(self, parent, theme, ctrl):
        self.parent = parent
        self.theme = theme
        self.ctrl = ctrl
        self.frame = None
        self.build()

    def build(self):
        raise NotImplementedError
    
    def update(self, telemetry: dict, connection: dict) -> None:
        pass

    def apply_theme(self, theme: dict) -> None:
        self.theme = theme

        if self.theme is not None:
            self.frame.config(bg = theme["panel_bg"], fg = theme["fg"],
                              highlightbackground  = theme["border"], bd = 1, relief = "solid",)