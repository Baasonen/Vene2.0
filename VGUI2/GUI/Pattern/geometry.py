import math
from typing import Tuple, List, Optional

from GUI.Pattern.constants import EARTH_RADIUS_M

# Move dist meters from (lat, lon) toward bearing
def project(lat: float, lon: float, bearing_deg: float, dist_m: float) -> Tuple[float, float]:
    theta = math.radians(bearing_deg)

    north_m = dist_m * math.cos(theta)
    east_m = dist_m * math.sin(theta)

    dlat = north_m / EARTH_RADIUS_M
    dlon = east_m / (EARTH_RADIUS_M * math.cos(math.radians(lat)))

    return lat + math.degrees(dlat), lon + math.degrees(dlon)

def local_offset_m(origin: Tuple[float, float], point: Tuple[float, float]) -> Tuple[float, float]:
    lat0, lon0 = origin
    lat, lon = point

    north_m = math.radians(lat - lat0) * EARTH_RADIUS_M
    east_m = math.radians(lon - lon0) * EARTH_RADIUS_M * math.cos(math.radians(lat0))

    return north_m, east_m

def distance_m(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    north_m, east_m = local_offset_m(a, b)

    return math.hypot(north_m, east_m)

def polygon_area_m2(points: List[Tuple[float, float]]) -> float:
    # NOTE: Flat-earth projection, prob. accurate enough for this use case
    if len(points) < 3:
        return 0.0 

    origin = points[0]
    local = [local_offset_m(origin, p) for p in points]

    area = 0.0
    n = len(local)

    for i in range(n):
        x1, y1 = local[i]
        x2, y2 = local[(i + 1) % n]

        area += x1 * y2 - x2 * y1

    return abs(area) / 2.0

# Vertex avg. good enough
def polygon_centroid(points: List[Tuple[float, float]]) -> Tuple[float, float]:
    lat = sum(p[0] for p in points) / len(points)
    lon = sum(p[1] for p in points) / len(points)

    return lat, lon

# Project point into the local plane rotated by theta (rad) around origin
def to_local(origin: Tuple[float, float], theta: float, point: Tuple[float, float]) -> Tuple[float, float]:
    sin_t, cos_t = math.sin(theta), math.cos(theta)
    north_m, east_m = local_offset_m(origin, point)

    along = north_m * cos_t + east_m * sin_t
    across = east_m * cos_t - north_m * sin_t

    return along, across

def from_local(origin: Tuple[float, float], theta: float, along: float, across: float) -> Tuple[float, float]:
    sin_t, cos_t = math.sin(theta), math.cos(theta)
    north_m = along * cos_t - across * sin_t
    east_m = along * sin_t + across * cos_t

    lat0, lon0 = origin
    dlat = math.degrees(north_m / EARTH_RADIUS_M)
    dlon = math.degrees(east_m / (EARTH_RADIUS_M * math.cos(math.radians(lat0))))

    return (lat0 + dlat, lon0 + dlon)

def _orientation(p: Tuple[float, float], q: Tuple[float, float], r: Tuple[float, float]) -> int:
    val = (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])

    if abs(val) < 1e-12:
        return 0

    return 1 if val > 0 else -1

def _on_segment(p: Tuple[float, float], q: Tuple[float, float], r: Tuple[float, float]) -> bool:
    return (min(p[0], r[0]) - 1e-9 <= q[0] <= max(p[0], r[0]) + 1e-9 and
            min(p[1], r[1]) - 1e-9 <= q[1] <= max(p[1], r[1]) + 1e-9)

def _segments_intersect(p1, p2, p3, p4) -> bool:
    o1, o2 = _orientation(p1, p2, p3), _orientation(p1, p2, p4)
    o3, o4 = _orientation(p3, p4, p1), _orientation(p3, p4, p2)

    if o1 != o2 and o3 != o4:
        return True

    if o1 == 0 and _on_segment(p1, p3, p2):
        return True
    if o2 == 0 and _on_segment(p1, p4, p2):
        return True
    if o3 == 0 and _on_segment(p3, p1, p4):
        return True
    if o4 == 0 and _on_segment(p3, p2, p4):
        return True

    return False

def find_self_intersection(points: List[Tuple[float, float]]) -> Optional[int]:
    n = len(points)

    if n < 4:
        return None

    edges = [(points[i], points[(i + 1) % n]) for i in range(n)]

    for i in range(n):
        for j in range(i + 1, n):
            if j == i + 1 or (i == 0 and j == n - 1):
                continue # Edges sharing a vertex aren't a "crossing"

            if _segments_intersect(edges[i][0], edges[i][1], edges[j][0], edges[j][1]):
                return j + 1

    return None

# Index of the polygon edge that pt lies on
def closest_edge(pt: Tuple[float, float], poly: List[Tuple[float, float]]) -> int:
    best_i = 0
    best_d = float('inf')
    n = len(poly)

    for i in range(n):
        p1 = poly[i]
        p2 = poly[(i + 1) % n]

        # Segment length squared
        l2 = (p1[0] - p2[0])**2 + (p1[1] - p2[1])**2
        if l2 == 0:
            continue

        # Project pt onto the segment p1-p2
        t = max(0, min(1, ((pt[0] - p1[0]) * (p2[0] - p1[0]) + (pt[1] - p1[1]) * (p2[1] - p1[1])) / l2))
        proj = (p1[0] + t * (p2[0] - p1[0]), p1[1] + t * (p2[1] - p1[1]))

        d = math.hypot(pt[0] - proj[0], pt[1] - proj[1])
        if d < best_d:
            best_d = d
            best_i = i

    return best_i

def path_length(path: List[Tuple[float, float]]) -> float:
    return sum(math.hypot(path[k][0] - path[k-1][0], path[k][1] - path[k-1][1]) for k in range(1, len(path)))

def _intervals_overlap(a1: float, b1: float, a2: float, b2: float) -> bool:
    return a1 <= b2 and a2 <= b1

# Shortest path along the polygon perimeter
def get_boundary_path(p1: Tuple[float, float], p2: Tuple[float, float], poly: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
    n = len(poly)
    i = closest_edge(p1, poly)
    j = closest_edge(p2, poly)

    if i == j:
        return [] # They are on the same edge

    # Forward trace
    path_fwd = []
    curr = (i + 1) % n
    while curr != (j + 1) % n:
        path_fwd.append(poly[curr])
        curr = (curr + 1) % n

    # Backward trace
    path_bwd = []
    curr = i
    while curr != j:
        path_bwd.append(poly[curr])
        curr = (curr - 1) % n

    full_fwd = [p1] + path_fwd + [p2]
    full_bwd = [p1] + path_bwd + [p2]

    return path_fwd if path_length(full_fwd) < path_length(full_bwd) else path_bwd

def haversine_distance(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    lat1, lon1 = a
    lat2, lon2 = b
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    x = math.sin(dphi / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dlambda / 2) ** 2
    return 2 * EARTH_RADIUS_M * math.asin(math.sqrt(x))

def _row_intersections(local_poly: List[Tuple[float, float]], across: float) -> List[float]:
    xs = []
    n = len(local_poly)

    for i in range(n):
        a1, c1 = local_poly[i]
        a2, c2 = local_poly[(i + 1) % n]

        if c1 == c2:
            continue # Ignore edge parallel to the row

        if (c1 <= across < c2) or (c2 <= across < c1):
            t = (across - c1) / (c2 - c1)
            xs.append(a1 + t * (a2 - a1))

    xs.sort()
    return xs