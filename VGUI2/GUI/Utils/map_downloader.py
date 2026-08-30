import os
import math
import threading
import sqlite3
import requests
from concurrent.futures import ThreadPoolExecutor
from typing import Callable, Tuple, Optional
from tkintermapview import decimal_to_osm

_MAIN_DIR = os.path.dirname(
    os.path.dirname(
        os.path.dirname(os.path.abspath(__file__))
    )
)

MAP_DIR = os.path.join(_MAIN_DIR, "Data", "offline_tiles.db")

TILE_USER = None
DOWNLOAD_THREADS = 10
REQUEST_TIMEOUT = 10
AVG_TILE_BYTES = 18000

Point = Tuple[float, float]

def _tile_bounds(top_l: Point, bottom_r: Point, zoom: int) -> Tuple[int, int, int, int]:
    x_a, y_a = decimal_to_osm(*top_l, zoom)
    x_b, y_b = decimal_to_osm(*bottom_r, zoom)

    x_min, x_max = math.floor(min(x_a, x_b)), math.ceil(max(x_a, x_b))
    y_min, y_max = math.floor(min(y_a, y_b)), math.ceil(max(y_a, y_b))

    return x_min, x_max, y_min, y_max

def iterate_tile_coords(top_l: Point, bottom_r: Point, zoom_min: int, zoom_max: int):
    zoom_min, zoom_max = min(zoom_min, zoom_max), max(zoom_min, zoom_max)

    for zoom in range(zoom_min, zoom_max + 1):
        x_min, x_max, y_min, y_max = _tile_bounds(top_l, bottom_r, zoom)

        for x in range(x_min, x_max + 1):
            for y in range(y_min, y_max + 1):
                yield zoom, x, y

def estimate_tiles(top_l: Point, bottom_r: Point, zoom_min: int, zoom_max: int) -> Tuple[int, int]:
    zoom_min, zoom_max = min(zoom_min, zoom_max), max(zoom_min, zoom_max)

    count = 0
    for zoom in range(zoom_min, zoom_max + 1):
        x_min, x_max, y_min, y_max = _tile_bounds(top_l, bottom_r, zoom)
        count += (x_max - x_min + 1) * (y_max - y_min + 1)

    return count, count * AVG_TILE_BYTES

def _ensure_db(db_cursor: sqlite3.Cursor) -> None:
    db_cursor.execute("""CREATE TABLE IF NOT EXISTS server (
                            url VARCHAR(300) PRIMARY KEY NOT NULL,
                            max_zoom INTEGER NOT NULL);""")
 
    db_cursor.execute("""CREATE TABLE IF NOT EXISTS tiles (
                            zoom INTEGER NOT NULL,
                            x INTEGER NOT NULL,
                            y INTEGER NOT NULL,
                            server VARCHAR(300) NOT NULL,
                            tile_image BLOB NOT NULL,
                            CONSTRAINT fk_server FOREIGN KEY (server) REFERENCES server (url),
                            CONSTRAINT pk_tiles PRIMARY KEY (zoom, x, y, server));""")
 
    db_cursor.execute("""CREATE TABLE IF NOT EXISTS sections (
                            position_a VARCHAR(100) NOT NULL,
                            position_b VARCHAR(100) NOT NULL,
                            zoom_a INTEGER NOT NULL,
                            zoom_b INTEGER NOT NULL,
                            server VARCHAR(300) NOT NULL,
                            CONSTRAINT fk_server FOREIGN KEY (server) REFERENCES server (url),
                            CONSTRAINT pk_tiles PRIMARY KEY (position_a, position_b, zoom_a, zoom_b, server));""")

def get_db_size(db_path: str = MAP_DIR) -> int:
    try:
        return os.path.getsize(db_path)
    except OSError:
        return 0

def clear_db(db_path: str = MAP_DIR) -> None:
    if not os.path.exists(db_path):
        return

    db_connection = sqlite3.connect(db_path, timeout = 30, isolation_level = None)
    try:
        db_cursor = db_connection.cursor()
        db_cursor.execute("DELETE FROM tiles;")
        db_cursor.execute("DELETE FROM sections;")
        db_cursor.execute("VACUUM;")
    finally:
        db_connection.close()

class MapDownloader:
    def __init__(self, tile_server: str, db_path: str = MAP_DIR, max_zoom: int = 19, num_threads: int = DOWNLOAD_THREADS):
        self.tile_server = tile_server
        self.db_path = db_path
        self.max_zoom = max_zoom
        self.num_threads = num_threads

        self._cancel_event = threading.Event()

    def cancel(self) -> None:
        self._cancel_event.set()

    def download(self, top_l: Point, bottom_r: Point, zoom_min: int, zoom_max: int,
                 on_progress: Optional[Callable[[int, int], None]] = None,
                 on_done: Optional[Callable[[bool, int, Optional[str]], None]] = None) -> None:

        tasks = list(iterate_tile_coords(top_l, bottom_r, zoom_min, zoom_max))
        total = len(tasks)
        saved = 0
        error = None

        try:
            os.makedirs(os.path.dirname(self.db_path), exist_ok = True)

            db_connection = sqlite3.connect(self.db_path, timeout = 30, check_same_thread = False)
            db_cursor = db_connection.cursor()
            _ensure_db(db_cursor)

            db_cursor.execute("SELECT 1 FROM server WHERE url = ?;", (self.tile_server,))
            if db_cursor.fetchone() is None:
                db_cursor.execute("INSERT INTO server (url, max_zoom) VALUES (?, ?);",
                                  (self.tile_server, self.max_zoom))
            db_connection.commit()

            db_lock = threading.Lock()

            def fetch_tile(task: Tuple[int, int, int]) -> None:
                nonlocal saved

                if self._cancel_event.is_set():
                    return

                zoom, x, y, = task

                with db_lock:
                    db_cursor.execute(
                        "SELECT 1 FROM tiles WHERE zoom=? AND x=? AND y=? AND server=?;",
                        (zoom, x, y, self.tile_server))
                    already_have = db_cursor.fetchone() is not None

                img_data = None
                if not already_have:
                    url = (self.tile_server.replace("{z}", str(zoom))
                                           .replace("{x}", str(x))
                                           .replace("{y}", str(y)))

                    try:
                        response = requests.get(url, headers = TILE_USER, timeout = REQUEST_TIMEOUT)
                        response.raise_for_status()
                        img_data = response.content
                    except Exception:
                        img_data = None

                with db_lock:
                    if img_data:
                        db_cursor.execute(
                            "INSERT OR IGNORE INTO tiles (zoom, x, y, server, tile_image) VALUES (?, ?, ?, ?, ?);",
                            (zoom, x, y, self.tile_server, img_data))
                        db_connection.commit()
 
                    saved += 1
                    if on_progress:
                        on_progress(saved, total)

            with ThreadPoolExecutor(max_workers = self.num_threads) as p:
                for _ in p.map(fetch_tile, tasks):
                    if self._cancel_event.is_set():
                        break

            zoom_lo, zoom_hi = min(zoom_min, zoom_max), max(zoom_min, zoom_max)
            db_cursor.execute(
                "INSERT OR IGNORE INTO sections (position_a, position_b, zoom_a, zoom_b, server) "
                "VALUES (?, ?, ?, ?, ?);",
                (str(top_l), str(bottom_r), zoom_lo, zoom_hi, self.tile_server))
            db_connection.commit()
            db_connection.close()   

        except Exception as e:
            error = str(e)         

        if on_done:
            on_done(not self._cancel_event.is_set() and error is None, saved, error)