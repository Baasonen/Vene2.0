import math
from typing import List, Tuple

from GUI.Pattern.geometry import (polygon_centroid, to_local, _row_intersections, _intervals_overlap,
                                  to_local, from_local, get_boundary_path, path_length, project)

PARAM_SPECS = {
    "spacing_m": {
        "label": "Track spacing",
        "default": 25.0,
        "slider": True,
        "min": 5.0,
        "max": 100.0,
        "step": 1.0,
        "fmt": "{:.0f}",
        "unit": "m",
    },
    "orientation_deg": {
        "label": "Orientation",
        "default": 0.0,
        "slider": True,
        "min": 0.0,
        "max": 359.0,
        "step": 1.0,
        "fmt": "{:.0f}",
        "unit": "\u00b0",
    },
    "initial_leg_m": {
        "label": "Initial leg length (m)",
        "default": "50",
        "slider": False,
    },
    "num_legs": {
        "label": "Number of legs",
        "default": "8",
        "slider": False,
    },
}

PATTERNS = {
    "Lawnmower": ["spacing_m", "orientation_deg"],
    "Expanding Square": ["initial_leg_m", "num_legs", "orientation_deg"],
}

def _stitch_lanes(lanes: List[dict], poly: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
    remaining = list(lanes)
    remaining.sort(key = lambda lane: (lane["points"][0][1], lane["points"][0][0]))

    current = remaining.pop(0)
    waypoints: List[Tuple[float, float]] = list(current["points"])

    while remaining:
        tail = waypoints[-1]
        best_i, best_reverse, best_dist = 0, False, None

        for i, lane in enumerate(remaining):
            for reverse, end_point in ((False, lane["points"][0]), (True, lane["points"][-1])):
                # Actual perimeter travel distance
                boundary_transit = get_boundary_path(tail, end_point, poly)
                full_transit = [tail] + boundary_transit + [end_point]
                dist = path_length(full_transit)

                if best_dist is None or dist < best_dist:
                    best_dist, best_i, best_reverse = dist, i, reverse

        lane = remaining.pop(best_i)

        # We already know the best route, re-generate it to append
        next_start = lane["points"][-1] if best_reverse else lane["points"][0]
        boundary_transit = get_boundary_path(tail, next_start, poly)
        waypoints.extend(boundary_transit)

        pts = list(reversed(lane["points"])) if best_reverse else lane["points"]
        # The transit above never includes next_start itself
        if waypoints and pts and pts[0] == waypoints[-1]:
            pts = pts[1:]
        waypoints.extend(pts)

    return waypoints

def generate_lawnmower(area_points: List[Tuple[float, float]], spacing_m: float, orientation_deg: float) -> List[Tuple[float, float]]:
    if len(area_points) < 3:
        raise ValueError("Define an area with at least 3 points first")

    if spacing_m <= 0:
        raise ValueError("Track spacing must be positive")

    origin = polygon_centroid(area_points)
    theta = math.radians(orientation_deg % 360)

    local_poly = [to_local(origin, theta, p) for p in area_points]
    across_values = [c for _, c in local_poly]
    across_min, across_max = min(across_values), max(across_values)

    if across_max - across_min <= 0:
        raise ValueError("Area has no extent along this orientation")

    # Nudge rows off across_min / max so they never land exactly on a vertex
    # Avoids tie-break glitches on jagged, many-vertex boundaries
    eps = max(spacing_m * 1e-4, 1e-6)
    across = across_min + eps
    last_across = across_max - eps

    open_lanes: List[dict] = []
    finished_lanes: List[dict] = []

    while across <= last_across:
        xs = _row_intersections(local_poly, across)
        segments = list(zip(xs[0::2], xs[1::2]))

        lanes_by_segment: dict = {}
        segments_by_lane: dict = {}

        for si, (a, b) in enumerate(segments):
            for li, candidate in enumerate(open_lanes):
                if _intervals_overlap(candidate["a"], candidate["b"], a, b):
                    lanes_by_segment.setdefault(si, []).append(li)
                    segments_by_lane.setdefault(li, []).append(si)

        next_lanes: List[dict] = []

        for si, (a, b) in enumerate(segments):
            partners = lanes_by_segment.get(si, [])
            li = partners[0] if len(partners) == 1 and len(segments_by_lane[partners[0]]) == 1 else None
            lane = open_lanes[li] if li is not None else {"points": [], "parity": 0}

            lane["a"], lane["b"] = a, b
            first, second = (b, a) if lane["parity"] % 2 else (a, b)
            lane["points"].append((first, across))
            lane["points"].append((second, across))
            lane["parity"] += 1

            next_lanes.append(lane)

        for li, candidate in enumerate(open_lanes):
            partners = segments_by_lane.get(li, [])
            if not (len(partners) == 1 and len(lanes_by_segment[partners[0]]) == 1):
                finished_lanes.append(candidate)

        open_lanes = next_lanes
        across += spacing_m

    finished_lanes.extend(open_lanes)

    if not finished_lanes:
        raise ValueError("No coverage rows fit inside this area at the current spacing/orientation")

    local_waypoints = _stitch_lanes(finished_lanes, local_poly)

    return [from_local(origin, theta, a, c) for a, c in local_waypoints]

def generate_expanding_square(area_points: List[Tuple[float, float]], initial_leg_m: float, num_legs: int, orientation_deg: float) -> List[Tuple[float, float]]:
    if len(area_points) < 3:
        raise ValueError("Define an area with at least 3 points first")

    if initial_leg_m <= 0:
        raise ValueError("Initial leg length must be positive")

    if num_legs < 2:
        raise ValueError("Need at least 2 legs")

    lat, lon = polygon_centroid(area_points)
    waypoints = [(lat, lon)]

    heading = orientation_deg % 360
    leg_len = initial_leg_m
    legs_at_len = 0

    for _ in range(num_legs):
        lat, lon = project(lat, lon, heading, leg_len)
        waypoints.append((lat, lon))

        heading = (heading + 90) % 360
        legs_at_len += 1

        if legs_at_len == 2:
            legs_at_len = 0
            leg_len += initial_leg_m

    return waypoints