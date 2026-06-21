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

        if self.theme is not None and self.frame is not None:
            config = {
                "bg": theme["panel_bg"],
                "highlightbackground": theme["border"],
                "bd": 1,
                "relief": "solid",
            }

            if "fg" in self.frame.keys():
                config["fg"] = theme["fg"]

            self.frame.config(**config)