#!/usr/bin/env python3
"""Add deterministic walking actors and physical proxies to an airport SDF.

The generated world uses only the locally packaged walk.dae asset. Routes are
piecewise-linear closed loops; this is scripted pedestrian traffic rather than
a crowd behaviour model. All motion uses simulation time in the Gazebo system.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET


ASSET_URI = "model://omslam_pedestrian/meshes/walk.dae"
ANIMATION_FACTOR = 4.153902689  # animation seconds / metre, from the source hips track
ACTOR_HEIGHT_OFFSET = 1.0  # same skin-origin convention as Gazebo's actor example
PROXY_HEIGHT = 1.75
PROXY_RADIUS = 0.30


def number(value: object, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{label} must be a number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{label} must be finite")
    return result


def scalar(parent: ET.Element, tag: str, value: object) -> ET.Element:
    child = ET.SubElement(parent, tag)
    child.text = str(value)
    return child


def route_from_json(raw: object, index: int) -> dict:
    if not isinstance(raw, dict):
        raise ValueError(f"pedestrian_routes[{index}] must be an object")
    name = raw.get("name", f"pedestrian_{index:02d}")
    if not isinstance(name, str) or not re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*", name):
        raise ValueError(f"invalid pedestrian name: {name!r}")
    speed = number(raw.get("speed", 1.0), f"{name}.speed")
    phase = number(raw.get("phase", 0.0), f"{name}.phase")
    if speed <= 0.0 or phase < 0.0:
        raise ValueError(f"{name}: speed must be positive and phase non-negative")
    values = raw.get("waypoints")
    if not isinstance(values, list):
        raise ValueError(f"{name}.waypoints must be a list of [x, y] metres")
    points: list[tuple[float, float]] = []
    for i, point in enumerate(values):
        if not isinstance(point, (list, tuple)) or len(point) != 2:
            raise ValueError(f"{name}.waypoints[{i}] must have exactly two coordinates")
        xy = (number(point[0], f"{name}.x"), number(point[1], f"{name}.y"))
        if not points or math.dist(points[-1], xy) > 1e-8:
            points.append(xy)
    if len(points) > 1 and math.dist(points[0], points[-1]) <= 1e-8:
        points.pop()
    if len(points) < 2:
        raise ValueError(f"{name}: at least two different waypoints are required")
    lengths = [math.dist(points[i], points[(i + 1) % len(points)])
               for i in range(len(points))]
    if any(length <= 1e-8 for length in lengths):
        raise ValueError(f"{name}: zero-length route segment")
    return {"name": name, "speed": speed, "phase": phase,
            "waypoints": points, "lengths": lengths, "length": sum(lengths)}


def initial_pose(route: dict) -> tuple[float, float, float]:
    remaining = (route["phase"] * route["speed"]) % route["length"]
    segment = 0
    while segment + 1 < len(route["lengths"]) and remaining >= route["lengths"][segment]:
        remaining -= route["lengths"][segment]
        segment += 1
    a = route["waypoints"][segment]
    b = route["waypoints"][(segment + 1) % len(route["waypoints"])]
    fraction = remaining / route["lengths"][segment]
    return (a[0] + (b[0] - a[0]) * fraction,
            a[1] + (b[1] - a[1]) * fraction,
            math.atan2(b[1] - a[1], b[0] - a[0]))


def add_person(world: ET.Element, plugin: ET.Element, route: dict) -> None:
    name = route["name"]
    proxy_name = f"{name}_collision"
    actor = ET.SubElement(world, "actor", {"name": name})
    # Motion is relative to this fixed origin. The native skin is hip-centred.
    scalar(actor, "pose", f"0 0 {ACTOR_HEIGHT_OFFSET} 0 0 0")
    skin = ET.SubElement(actor, "skin")
    scalar(skin, "filename", ASSET_URI)
    scalar(skin, "scale", "1.0")
    animation = ET.SubElement(actor, "animation", {"name": "walk"})
    scalar(animation, "filename", ASSET_URI)
    scalar(animation, "scale", "1.0")
    scalar(animation, "interpolate_x", "true")
    script = ET.SubElement(actor, "script")
    scalar(script, "loop", "true")
    scalar(script, "auto_start", "false")

    # This top-level body is intentionally invisible: the animated mesh is
    # what RGB / GPU LiDAR see; the cylinder is only the physical approximation.
    proxy = ET.SubElement(world, "model", {"name": proxy_name})
    scalar(proxy, "static", "true")
    x, y, yaw = initial_pose(route)
    scalar(proxy, "pose", f"{x:.9g} {y:.9g} 0 0 0 {yaw:.9g}")
    link = ET.SubElement(proxy, "link", {"name": "body"})
    collision = ET.SubElement(link, "collision", {"name": "body_collision"})
    scalar(collision, "pose", f"0 0 {PROXY_HEIGHT / 2:.9g} 0 0 0")
    cylinder = ET.SubElement(ET.SubElement(collision, "geometry"), "cylinder")
    scalar(cylinder, "radius", PROXY_RADIUS)
    scalar(cylinder, "length", PROXY_HEIGHT)

    entry = ET.SubElement(plugin, "pedestrian")
    scalar(entry, "actor", name)
    scalar(entry, "collision_model", proxy_name)
    scalar(entry, "speed", route["speed"])
    scalar(entry, "phase", route["phase"])
    scalar(entry, "animation_factor", ANIMATION_FACTOR)
    for px, py in route["waypoints"]:
        scalar(entry, "waypoint", f"{px:.9g} {py:.9g}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", type=Path, required=True,
                        help="Input static SDF world")
    parser.add_argument("--layout", type=Path, required=True,
                        help="JSON containing pedestrian_routes")
    parser.add_argument("--output", type=Path, required=True,
                        help="Output SDF, normally airport.sdf")
    parser.add_argument("--max-pedestrians", type=int, default=64,
                        help="Use the first N routes (0 disables people, maximum 64)")
    args = parser.parse_args()
    if not 0 <= args.max_pedestrians <= 64:
        parser.error("--max-pedestrians must be between 0 and 64")
    try:
        if args.world.resolve() == args.output.resolve():
            raise ValueError("input static world and output must be different files")
        layout = json.loads(args.layout.read_text())
        if not isinstance(layout, dict):
            raise ValueError("layout JSON must be an object")
        raw_routes = layout.get("pedestrian_routes", [])
        if not isinstance(raw_routes, list):
            raise ValueError("pedestrian_routes must be a list")
        routes = [route_from_json(value, i) for i, value in
                  enumerate(raw_routes[:args.max_pedestrians])]
        tree = ET.parse(args.world)
        world = tree.getroot().find("world")
        if world is None:
            raise ValueError("input SDF must contain a world")
        names = {element.get("name") for element in world
                 if element.tag in ("model", "actor", "light")}
        for route in routes:
            for name in (route["name"], f"{route['name']}_collision"):
                if name in names:
                    raise ValueError(f"duplicate world entity name: {name}")
                names.add(name)
        if world.find("plugin[@name='omslam::AirportPedestrians']") is not None:
            raise ValueError("input world already contains the pedestrian plugin")
        if routes:
            plugin = ET.SubElement(world, "plugin", {
                "filename": "omslam_airport_pedestrians",
                "name": "omslam::AirportPedestrians"})
            scalar(plugin, "update_rate", "30")
            for route in routes:
                add_person(world, plugin, route)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        ET.indent(tree, space="  ")
        tree.write(args.output, encoding="utf-8", xml_declaration=True)
        print(json.dumps({"world": world.get("name"), "output": str(args.output),
                          "pedestrians": len(routes), "collision_proxies": len(routes),
                          "mesh_uri": ASSET_URI,
                          "diagnostics_topic": "/airport/pedestrians/poses"},
                         ensure_ascii=False))
    except (OSError, ValueError, ET.ParseError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    main()
