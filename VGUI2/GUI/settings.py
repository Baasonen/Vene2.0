import os

_MAIN_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SETTINGS_PATH = os.path.join(_MAIN_DIR, "Settings/settings.txt")

DEFAULT_SETTINGS = {
    "theme": "dark",
    "port": "COM4",
}

def load_settings() -> dict:
    settings = dict(DEFAULT_SETTINGS)

    try:
        with open(SETTINGS_PATH, "r", encoding = "utf-8") as f:
            for line in f:
                line = line.strip()

                if not line or line.startswith("#") or "=" not in line:
                    continue

                key, _, value = line.partition("=")
                settings[key.strip()] = value.strip()

    except FileNotFoundError:
        pass

    except OSError:
        pass

    return settings

def save_settings(settings: dict) -> bool:
    try:
        with open(SETTINGS_PATH, "w", encoding = "utf-8") as f:
            f.write("# VGUI2 Settings\n")

            for key, value in settings.items():
                f.write(f"{key}={value}\n")

        return True

    except OSError:
        return False