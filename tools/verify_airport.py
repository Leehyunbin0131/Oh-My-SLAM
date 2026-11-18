#!/usr/bin/env python3
"""One bounded acceptance run for the installed airport world.

Uses geometry/reference maps only as test truth. It does not navigate a robot,
contact hardware, or claim that the reference map was produced by SLAM.
"""
from __future__ import annotations

import argparse
from collections import deque
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import threading
import time
import traceback
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image as PillowImage
from scipy import ndimage
import yaml


def stamp(value):
    nanoseconds = value.nsec if hasattr(value, "nsec") else value.nanosec
    return float(value.sec) + float(nanoseconds) * 1e-9


def progress(message):
    print(message, flush=True)


def isolate_runtime_environment(attach=False):
    """Set isolation before constructing any Gazebo or ROS transport node."""
    if not attach:
        partition = f"omslam_airport_verify_{os.getpid()}"
        os.environ["GZ_PARTITION"] = partition
        os.environ["OMSLAM_GZ_PARTITION"] = partition
        os.environ["ROS_DOMAIN_ID"] = str(40+os.getpid()%160)
        os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def functional_fingerprint(share):
    """Only state relevant to passed physics/connectivity/sensor checks.

    Visual materials may change; collision transforms, actor assets, routes,
    bridge settings and the loaded motion plugin must remain byte-identical or
    structurally identical before those checks can be reused.
    """
    share = Path(share)
    world = ET.parse(share/"worlds/airport.sdf").find("world")
    canonical = lambda element: ET.canonicalize(ET.tostring(element, encoding="unicode"), strip_text=True)
    collisions = []
    for model in world.findall("model"):
        for link in model.findall("link"):
            for collision in link.findall("collision"):
                collisions.append({"model": model.get("name"), "model_pose": model.findtext("pose", ""),
                                   "static": model.findtext("static", ""), "link": link.get("name"),
                                   "link_pose": link.findtext("pose", ""), "collision": canonical(collision)})
    layout = json.loads((share/"config/airport_layout.json").read_text())
    selected = {key: layout[key] for key in ("bounds", "floors", "terminals", "corridors", "obstacles", "spawn", "pedestrian_routes")}
    hash_object = lambda value: hashlib.sha256(json.dumps(value, sort_keys=True).encode()).hexdigest()
    actor_root = share/"models/omslam_pedestrian"
    return {
        "collision_geometry": hash_object(collisions),
        "layout_geometry": hash_object(selected),
        "actors_and_world_systems": hash_object([canonical(n) for n in world if n.tag in ("actor", "plugin", "physics", "gravity")]),
        "actor_assets": {str(p.relative_to(actor_root)): digest(p) for p in sorted(actor_root.rglob("*")) if p.is_file()},
        "motion_plugin": digest(share.parent.parent/"lib/libomslam_airport_pedestrians.so"),
        "clock_bridge": digest(share/"config/airport_bridge.yaml"),
        "reference_map": digest(share/"maps/airport_reference.pgm"),
    }


def appearance_metrics(image):
    """Scene-specific regression for the observed all-white airport defect.

    This checks that expected colour/structure exists; it is not an aesthetic
    score or a replacement for viewing the rendered images and signage.
    """
    pixels = np.asarray(image, dtype=np.float32)
    height, width = pixels.shape[:2]
    sky = np.median(pixels[:max(1, height//12)].reshape(-1, 3), axis=0)
    roi = pixels[height//10:height*9//10, width//20:width*19//20]
    surface = np.max(np.abs(roi-sky), axis=2) > 15
    count = max(int(surface.sum()), 1)
    span = roi.max(axis=2)-roi.min(axis=2)
    light = roi.mean(axis=2)
    white = (roi.min(axis=2) >= 245) & (span < 12)
    colour = (span >= 35) & (roi.max(axis=2) >= 70) & (light < 235)
    dark = light < 150
    edge = np.zeros(surface.shape, dtype=bool)
    edge[1:] |= np.max(np.abs(roi[1:]-roi[:-1]), axis=2) >= 25
    edge[:, 1:] |= np.max(np.abs(roi[:, 1:]-roi[:, :-1]), axis=2) >= 25
    values = {"surface_pixels": count, "surface_view_fraction": float(count/surface.size),
              "white_surface_fraction": float((white & surface).sum()/count),
              "coloured_surface_fraction": float((colour & surface).sum()/count),
              "dark_structure_fraction": float((dark & surface).sum()/count),
              "detail_edge_fraction": float((edge & surface).sum()/count)}
    passed = (values["surface_view_fraction"] >= 0.20 and values["white_surface_fraction"] < 0.85
              and values["coloured_surface_fraction"] >= 0.005
              and values["dark_structure_fraction"] >= 0.005 and values["detail_edge_fraction"] >= 0.006)
    return {"passes_known_blank_material_regression": passed, **values}


def point_segment_distance(point, a, b):
    p, a, b = np.asarray(point), np.asarray(a), np.asarray(b)
    d = b - a
    u = np.clip(np.dot(p-a, d) / max(np.dot(d, d), 1e-15), 0, 1)
    return float(np.linalg.norm(p-(a+u*d)))


def inside_polygon(x, y, vertices):
    inside = False
    for a, b in zip(vertices, vertices[1:]+vertices[:1]):
        if (a[1] > y) != (b[1] > y):
            boundary = (b[0]-a[0])*(y-a[1])/(b[1]-a[1])+a[0]
            if x < boundary:
                inside = not inside
    return inside


def ray_polygon_distance(origin, angle, vertices):
    """Nearest intersection of a forward XY ray with a polygon boundary."""
    direction = np.array([math.cos(angle), math.sin(angle)])
    origin = np.asarray(origin)
    nearest = math.inf
    if inside_polygon(*origin, vertices):
        return 0.0
    cross = lambda a, b: a[0]*b[1]-a[1]*b[0]
    for a, b in zip(vertices, vertices[1:]+vertices[:1]):
        a, b = np.asarray(a), np.asarray(b)
        edge = b-a
        denominator = cross(direction, edge)
        if abs(denominator) < 1e-10:
            continue
        t = cross(a-origin, edge)/denominator
        u = cross(a-origin, direction)/denominator
        if t >= 0 and 0 <= u <= 1:
            nearest = min(nearest, float(t))
    return nearest


class Results:
    def __init__(self, directory):
        self.directory = Path(directory).resolve()
        self.directory.mkdir(parents=True, exist_ok=True)
        self.data = {"status": "in_progress", "checks": {}, "scope": "synthetic airport world"}

    def check(self, key, passed, **evidence):
        self.data["checks"][key] = {"status": "PASS" if passed else "FAIL", **evidence}
        progress(f"{key}: {'PASS' if passed else 'FAIL'}")
        self.save()

    def save(self):
        (self.directory/"results.json").write_text(json.dumps(self.data, ensure_ascii=False, indent=2)+"\n")


class Geometry:
    def __init__(self, share):
        self.share = Path(share)
        self.layout_path = self.share/"config/airport_layout.json"
        self.layout = json.loads(self.layout_path.read_text())
        map_yaml = self.share/"maps/airport_reference.yaml"
        metadata = yaml.safe_load(map_yaml.read_text())
        self.resolution = float(metadata["resolution"])
        self.origin = metadata["origin"][:2]
        pixels = np.asarray(PillowImage.open(map_yaml.parent/metadata["image"]).convert("L"))
        self.free = np.flipud(pixels >= 250)
        self.clearance = ndimage.distance_transform_edt(self.free)*self.resolution
        self.robot_free = self.clearance >= 0.30
        self.labels, self.component_count = ndimage.label(self.robot_free)
        self.obstacles = [o for o in self.layout["obstacles"]
                          if float(o["z_max"]) > 0.02 and float(o["z_min"]) < 1.9]

    def cell(self, xy):
        return (int((xy[1]-self.origin[1])/self.resolution),
                int((xy[0]-self.origin[0])/self.resolution))

    def xy(self, cell):
        return [self.origin[0]+(cell[1]+0.5)*self.resolution,
                self.origin[1]+(cell[0]+0.5)*self.resolution]

    def clearance_at(self, xy):
        r, c = self.cell(xy)
        return float(self.clearance[r, c]) if 0 <= r < self.free.shape[0] and 0 <= c < self.free.shape[1] else 0.0

    def static_range(self, xy, angle, height=0.9):
        return min((ray_polygon_distance(xy, angle, o["footprint"]) for o in self.obstacles
                    if o["z_min"] <= height <= o["z_max"]), default=math.inf)

    def world_structure_check(self, result, minimum_wide_corridor=6.4,
                              expected_terminals=("T1", "T2"), expected_pedestrians=None,
                              expected_bounds=None):
        world_path = self.share/"worlds/airport.sdf"
        tree = ET.parse(world_path)
        world = tree.find("world")
        terminals = self.layout["terminals"]
        widths = [float(c.get("width", c.get("width_m", 0))) for c in self.layout["corridors"]]
        collisions, visuals = len(world.findall(".//collision")), len(world.findall(".//visual"))
        mesh_uris = [n.text.strip() for n in world.findall(".//visual/geometry/mesh/uri") if n.text]
        missing_meshes = []
        for uri in mesh_uris:
            path = self.share/"models"/uri[len("model://"):] if uri.startswith("model://") else Path(uri.removeprefix("file://"))
            if not path.is_file() or not path.stat().st_size:
                missing_meshes.append(uri)
        actors = len(world.findall("actor"))
        terminal_ids = [terminal["id"] for terminal in terminals]
        bounds = self.layout["bounds"]
        outside = []
        for obstacle in self.obstacles:
            if any(x < bounds[0]-0.5 or x > bounds[2]+0.5 or y < bounds[1]-0.5 or y > bounds[3]+0.5
                   for x, y in obstacle["footprint"]):
                outside.append(obstacle["name"])
        correct_bounds = expected_bounds is None or np.allclose(bounds, expected_bounds, atol=1e-8)
        result.check("world_structure", world is not None and sorted(terminal_ids) == sorted(expected_terminals)
                     and min(widths) <= 1.6 and max(widths) >= minimum_wide_corridor and collisions > 50 and visuals > 0
                     and mesh_uris and not missing_meshes
                     and actors == len(self.layout["pedestrian_routes"])
                     and (expected_pedestrians is None or actors == expected_pedestrians)
                     and correct_bounds and not outside,
                     terminals=terminal_ids, expected_terminals=list(expected_terminals), bounds=bounds,
                     expected_bounds=expected_bounds, out_of_bounds_obstacles=outside,
                     corridor_widths_m=sorted(set(widths)), collisions=collisions, visuals=visuals,
                     mesh_uri_count=len(mesh_uris), missing_meshes=missing_meshes,
                     actors=actors, sdf_sha256=digest(world_path), minimum_wide_corridor_m=minimum_wide_corridor)

    def static_checks(self, result, minimum_wide_corridor=6.4,
                      expected_terminals=("T1", "T2"), expected_pedestrians=None,
                      expected_bounds=None):
        self.world_structure_check(result, minimum_wide_corridor, expected_terminals,
                                   expected_pedestrians, expected_bounds)
        world_path = self.share/"worlds/airport.sdf"
        terminals = self.layout["terminals"]
        spawn = self.layout["spawn"]
        spawn_cell = self.cell([spawn["x"], spawn["y"]])
        spawn_component = int(self.labels[spawn_cell])
        reachable = []
        for terminal in terminals:
            xmin, ymin, xmax, ymax = terminal["bounds"]
            r0, c0 = self.cell([xmin+1, ymin+1])
            r1, c1 = self.cell([xmax-1, ymax-1])
            area = self.labels[max(r0, 0):r1+1, max(c0, 0):c1+1]
            valid = area > 0
            reached = (area == spawn_component) & valid
            reachable.append({"id": terminal["id"], "reachable_free_area_m2": float(reached.sum()*self.resolution**2),
                              "reachable_fraction": float(reached.sum()/max(valid.sum(), 1))})
        result.check("terminal_connectivity", spawn_component > 0 and all(t["reachable_free_area_m2"] >= 100
                     and t["reachable_fraction"] >= 0.90 for t in reachable),
                     clearance_radius_m=0.30, interpretation="0.56 m width clearance; not an SE2 robot-navigation test",
                     spawn=spawn, spawn_clearance_m=self.clearance_at([spawn["x"], spawn["y"]]), terminals=reachable)
        route_results = []
        for route in self.layout["pedestrian_routes"]:
            points = route["waypoints"]
            sample_clearances = []
            for a, b in zip(points, points[1:]+points[:1]):
                distance = math.dist(a, b)
                for u in np.linspace(0, 1, max(2, math.ceil(distance/0.10)+1)):
                    sample_clearances.append(self.clearance_at(np.asarray(a)*(1-u)+np.asarray(b)*u))
            route_results.append({"name": route["name"], "minimum_static_clearance_m": min(sample_clearances),
                                  "sample_count": len(sample_clearances)})
        result.check("pedestrian_route_clearance", len(route_results) >= 3 and all(
            r["minimum_static_clearance_m"] >= 0.30 for r in route_results), routes=route_results)
        result.data["manifest"] = {"package_share": str(self.share), "layout_sha256": digest(self.layout_path),
                                   "world_sha256": digest(world_path), "reference_map_origin": self.origin,
                                   "reference_map_sha256": digest(self.share/"maps/airport_reference.pgm"),
                                   "reference_map_resolution_m": self.resolution,
                                   "reference_map_kind": "geometry-derived test reference, not SLAM output"}
        result.save()

    def density_statistics(self):
        terminals = []
        for terminal in self.layout["terminals"]:
            xmin, ymin, xmax, ymax = terminal["bounds"]
            r0, c0 = self.cell([xmin, ymin]); r1, c1 = self.cell([xmax, ymax])
            free = self.free[max(0, r0):r1+1, max(0, c0):c1+1]
            clearance = self.clearance[max(0, r0):r1+1, max(0, c0):c1+1]
            values = clearance[free]
            entry = {"id": terminal["id"], "free_area_m2": float(free.sum()*self.resolution**2),
                     "clearance_max_m": float(values.max()), "clearance_p95_m": float(np.percentile(values, 95))}
            for radius in (2.0, 3.0):
                empty = free & (clearance >= radius)
                labels, count = ndimage.label(empty)
                sizes = np.bincount(labels.ravel())[1:]
                entry[f"open_regions_clearance_{radius:g}m"] = {
                    "region_count": int(count), "total_area_m2": float(empty.sum()*self.resolution**2),
                    "largest_area_m2": float(sizes.max()*self.resolution**2) if sizes.size else 0.0}
            terminals.append(entry)
        stats = self.layout.get("statistics", {})
        return {"boarding_seats": stats.get("boarding_seating_capacity"),
                "collision_shapes": stats.get("collision_shapes", len(self.layout["obstacles"])),
                "pedestrians": len(self.layout["pedestrian_routes"]), "terminals": terminals,
                "method": "Static reference occupancy grid; clearance to nearest occupied/unknown cell. High-clearance regions are connected free pixels at the stated radius; not semantic crowd-capacity estimates."}

    def density_comparison(self, before_share, result, expected_pedestrians=None, minimum_seats=None):
        before = Geometry(before_share).density_statistics()
        after = self.density_statistics()
        ratios = []
        for old, new in zip(before["terminals"], after["terminals"]):
            if old["id"] != new["id"]:
                raise ValueError("Density comparison terminal IDs differ")
            ratios.append({"id": new["id"], "clearance_p95_ratio": new["clearance_p95_m"]/old["clearance_p95_m"],
                           "clearance_max_ratio": new["clearance_max_m"]/old["clearance_max_m"],
                           "large_open_area_2m_ratio": new["open_regions_clearance_2m"]["total_area_m2"] /
                           max(old["open_regions_clearance_2m"]["total_area_m2"], self.resolution**2)})
        passed = (after["boarding_seats"] >= before["boarding_seats"]*2 and
                  after["collision_shapes"] > before["collision_shapes"] and
                  all(r["clearance_p95_ratio"] < 1.0 and r["large_open_area_2m_ratio"] < 1.0 for r in ratios))
        if expected_pedestrians is not None:
            passed = passed and after["pedestrians"] == expected_pedestrians
        if minimum_seats is not None:
            passed = passed and after["boarding_seats"] >= minimum_seats
        result.check("density_improvement", passed, before=before, after=after, per_terminal_ratios=ratios,
                     acceptance="At least twice previous seats, more collision obstacles, lower p95 clearance and less >=2m-clearance open area in every terminal; retain separate width/connectivity check.",
                     expected_pedestrians=expected_pedestrians, minimum_seats=minimum_seats)


class Simulator:
    def __init__(self, geometry, result, attach=False, startup_timeout=120, render_only=False):
        isolate_runtime_environment(attach)
        from gz.transport13 import Node
        from gz.msgs10.world_stats_pb2 import WorldStatistics
        from gz.msgs10.pose_v_pb2 import Pose_V
        self.geometry, self.result = geometry, result
        self.render_only = render_only
        self.node = Node()
        self.lock = threading.Lock()
        history_size = max(3000, len(geometry.layout["pedestrian_routes"])*2*5*60)
        self.stats, self.poses, self.pose_history = deque(maxlen=1000), {}, deque(maxlen=history_size)
        self.sensor_samples = {}
        self.created = []
        self.process = None
        self.log_handle = None
        self.ros_context = None
        self.ros_node = None
        self.ros_executor = None
        self.ros_clock = deque(maxlen=1000)
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from rosgraph_msgs.msg import Clock
        self.rclpy = rclpy
        if not render_only:
            self.ros_context = Context()
            rclpy.init(context=self.ros_context)
            self.ros_node = rclpy.create_node("airport_world_validation", context=self.ros_context)
            self.ros_executor = SingleThreadedExecutor(context=self.ros_context)
            self.ros_executor.add_node(self.ros_node)
            self.ros_node.create_subscription(Clock, "/clock", lambda m: self.ros_clock.append(stamp(m.clock)),
                                             QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                                        durability=DurabilityPolicy.VOLATILE))
        self.node.subscribe(WorldStatistics, "/world/airport/stats", self.on_stats)
        self.node.subscribe(Pose_V, "/airport/pedestrians/poses", self.on_poses)
        if not attach:
            self.log_handle = (result.directory/"launch.log").open("w")
            command = ["ros2", "launch", "omslam_sim", "airport.launch.py", "gui:=false", "pedestrians:=true"]
            self.process = subprocess.Popen(command, stdout=self.log_handle, stderr=subprocess.STDOUT,
                                            start_new_session=True, env=os.environ.copy())
            result.data["launch"] = {"command": command, "pid": self.process.pid,
                                      "gz_partition": os.environ.get("GZ_PARTITION"),
                                      "ros_domain_id": os.environ.get("ROS_DOMAIN_ID"),
                                      "ros_automatic_discovery_range": os.environ.get("ROS_AUTOMATIC_DISCOVERY_RANGE")}
        try:
            self.until(lambda: len(self.stats) >= 2 and self.stats[-1][0] > self.stats[0][0], startup_timeout,
                       "world time progressing")
            if not render_only:
                result.check("gazebo_time", True, initial_sim_s=self.stats[0][0], observed_sim_s=self.stats[-1][0])
                self.until(lambda: len(self.ros_clock) >= 2 and self.ros_clock[-1] > self.ros_clock[0], 20,
                           "ROS clock bridge updating")
                result.check("ros_clock_bridge", True, initial_sim_s=self.ros_clock[0], latest_sim_s=self.ros_clock[-1],
                             samples=len(self.ros_clock), topic="/clock")
        except Exception:
            self.close()
            raise

    def on_stats(self, message):
        with self.lock:
            self.stats.append((stamp(message.sim_time), time.monotonic(), float(message.real_time_factor)))

    def on_poses(self, message):
        t = stamp(message.header.stamp)
        with self.lock:
            for p in message.pose:
                item = (t, p.position.x, p.position.y, p.position.z)
                self.poses[p.name] = item
                self.pose_history.append((p.name, *item))

    def until(self, condition, seconds, description):
        deadline, next_note = time.monotonic()+seconds, time.monotonic()+15
        while not condition():
            if self.ros_executor is not None:
                self.ros_executor.spin_once(timeout_sec=0.0)
            if self.process is not None and self.process.poll() is not None:
                raise RuntimeError(f"Gazebo launch exited {self.process.returncode}: {description}")
            if time.monotonic() >= deadline:
                raise TimeoutError(description)
            if time.monotonic() >= next_note:
                progress(f"Waiting: {description}")
                next_note += 15
            time.sleep(0.05)

    def create(self, name, sdf):
        from gz.msgs10.entity_factory_pb2 import EntityFactory
        from gz.msgs10.boolean_pb2 import Boolean
        request = EntityFactory(sdf=sdf, name=name, allow_renaming=False)
        ok, response = self.node.request("/world/airport/create", request, EntityFactory, Boolean, 5000)
        if not ok or not response.data:
            raise RuntimeError(f"Could not create sensor model {name}")
        self.created.append(name)

    def remove(self, name):
        from gz.msgs10.entity_pb2 import Entity
        from gz.msgs10.boolean_pb2 import Boolean
        request = Entity(name=name, type=Entity.MODEL)
        try:
            self.node.request("/world/airport/remove", request, Entity, Boolean, 2000)
            self.created.remove(name)
        except Exception:
            pass

    def subscribe_sensor(self, topic, cls, capacity=100):
        with self.lock:
            self.sensor_samples[topic] = deque(maxlen=capacity)
        def receive(message):
            with self.lock:
                self.sensor_samples[topic].append(message)
        self.node.subscribe(cls, topic, receive)

    def camera(self, name, pose, hfov=1.0, width=1280, height=720, clip_seconds=0):
        from gz.msgs10.image_pb2 import Image
        topic = f"/airport/verification/{name}/image"
        self.subscribe_sensor(topic, Image, capacity=60)
        model_name = "verification_camera_"+name
        pose_text = " ".join(str(v) for v in pose)
        sdf = f'''<sdf version="1.9"><model name="{model_name}"><static>true</static><pose>{pose_text}</pose>
          <link name="camera_link"><sensor name="camera" type="camera"><always_on>true</always_on>
          <update_rate>{5 if clip_seconds else 1}</update_rate><topic>{topic}</topic><camera>
          <horizontal_fov>{hfov}</horizontal_fov><image><width>{width}</width><height>{height}</height><format>R8G8B8</format></image>
          <clip><near>0.05</near><far>500</far></clip></camera></sensor></link></model></sdf>'''
        self.create(model_name, sdf)
        def nonblank():
            queue = self.sensor_samples[topic]
            return bool(queue) and np.frombuffer(queue[-1].data, dtype=np.uint8).std() > 5
        self.until(nonblank, 60, f"nonblank camera {name}")
        first = self.sensor_samples[topic][-1]
        target_stamp = stamp(first.header.stamp)+clip_seconds
        if clip_seconds:
            self.until(lambda: stamp(self.sensor_samples[topic][-1].header.stamp) >= target_stamp, 90,
                       f"walking camera clip {name}")
        messages = list(self.sensor_samples[topic])
        last = messages[-1]
        def decode(msg):
            if len(msg.data) < msg.height*msg.step or msg.step < msg.width*3:
                raise RuntimeError(f"Unexpected camera pixel buffer for {name}")
            return PillowImage.frombytes("RGB", (msg.width, msg.height), msg.data, "raw", "RGB", msg.step)
        path = self.result.directory/(name+".png")
        decode(last).save(path)
        evidence = {"image": str(path), "topic": topic, "size": [last.width, last.height],
                    "sim_time_s": stamp(last.header.stamp), "pose": pose,
                    "pixel_std": float(np.frombuffer(last.data, dtype=np.uint8).std()),
                    "appearance": appearance_metrics(decode(last))}
        if clip_seconds:
            selected = [m for m in messages if stamp(m.header.stamp) >= stamp(first.header.stamp)]
            frames = self.result.directory/(name+"_frames")
            frames.mkdir(exist_ok=True)
            for index, message in enumerate(selected):
                decode(message).save(frames/f"frame_{index:04d}.png")
            clip = self.result.directory/(name+".mp4")
            command = ["ffmpeg", "-hide_banner", "-loglevel", "error", "-y", "-framerate", "5",
                       "-i", str(frames/"frame_%04d.png"), "-c:v", "libx264", "-pix_fmt", "yuv420p", str(clip)]
            encoded = subprocess.run(command, capture_output=True, text=True, timeout=30)
            evidence.update({"frame_count": len(selected), "video": str(clip) if encoded.returncode == 0 else None,
                             "video_encode_error": encoded.stderr if encoded.returncode else None,
                             "frame_difference_mean": float(np.abs(np.asarray(decode(selected[0]), dtype=float)-
                                                                   np.asarray(decode(selected[-1]), dtype=float)).mean())})
        # Keep cameras until owned-world shutdown. On the installed Ogre2/Mesa
        # stack, repeated sensor deletion followed by creation triggered a
        # renderer-thread segmentation fault during this acceptance run.
        if self.process is None:
            self.remove(model_name)
        progress(f"Saved image: {path}")
        return evidence

    def verify_motion(self):
        names = [r["name"] for r in self.geometry.layout["pedestrian_routes"]]
        self.until(lambda: all(n in self.poses and n+"_collision" in self.poses for n in names), 30,
                   "actor and collision proxy observations")
        first = {n: self.poses[n] for n in names}
        initial_time = self.stats[-1][0]
        self.until(lambda: self.stats[-1][0] >= initial_time+3, 60, "pedestrian movement interval")
        checks = []
        for name in names:
            with self.lock:
                actor, proxy = self.poses[name], self.poses[name+"_collision"]
            route = next(r for r in self.geometry.layout["pedestrian_routes"] if r["name"] == name)
            points = route["waypoints"]
            route_error = min(point_segment_distance(actor[1:3], a, b) for a, b in zip(points, points[1:]+points[:1]))
            checks.append({"name": name, "displacement_m": math.dist(first[name][1:3], actor[1:3]),
                           "actor_proxy_xy_error_m": math.dist(actor[1:3], proxy[1:3]),
                           "route_error_m": route_error, "static_clearance_m": self.geometry.clearance_at(actor[1:3]),
                           "actor": actor, "proxy": proxy})
        passed = all(c["displacement_m"] > 0.4 and c["actor_proxy_xy_error_m"] <= 0.06
                     and c["route_error_m"] <= 0.10 and c["static_clearance_m"] >= 0.30 for c in checks)
        self.result.check("pedestrian_motion_and_proxy", passed, actors=checks,
                          source="ECM Actor Pose*TrajectoryPose and post-physics proxy worldPose")

    def sensor_people(self):
        from gz.msgs10.laserscan_pb2 import LaserScan
        names = [r["name"] for r in self.geometry.layout["pedestrian_routes"]]
        chosen = None
        for name in names:
            p = self.poses[name]
            for angle in np.arange(0, 2*math.pi, math.pi/8):
                position = [p[1]-3.5*math.cos(angle), p[2]-3.5*math.sin(angle)]
                if self.geometry.clearance_at(position) >= 0.7 and self.geometry.static_range(position, angle) > 6:
                    chosen = name, position, float(angle)
                    break
            if chosen:
                break
        if chosen is None:
            raise RuntimeError("No unobstructed 3.5 m pedestrian sensor view found")
        name, position, angle = chosen
        model_name, topic = "verification_people_lidar", "/airport/verification/people/scan"
        self.subscribe_sensor(topic, LaserScan)
        sdf = f'''<sdf version="1.9"><model name="{model_name}"><static>true</static>
        <pose>{position[0]} {position[1]} 0.9 0 0 {angle}</pose><link name="lidar_link">
        <sensor name="lidar" type="gpu_lidar"><always_on>true</always_on><update_rate>10</update_rate><topic>{topic}</topic>
        <lidar><scan><horizontal><samples>361</samples><resolution>1</resolution><min_angle>-0.9</min_angle><max_angle>0.9</max_angle></horizontal>
        <vertical><samples>1</samples><min_angle>0</min_angle><max_angle>0</max_angle></vertical></scan>
        <range><min>0.1</min><max>30</max><resolution>0.01</resolution></range></lidar></sensor></link></model></sdf>'''
        self.create(model_name, sdf)
        self.until(lambda: len(self.sensor_samples[topic]) >= 3, 60, "GPU lidar frames")
        hits = []
        deadline = time.monotonic()+20
        consumed = None
        while time.monotonic() < deadline and len(hits) < 3:
            scans = self.sensor_samples[topic]
            if scans and scans[-1] is not consumed:
                scan = scans[-1]
                consumed = scan
                for actor_name in names:
                    actor = self.poses[actor_name]
                    dx, dy = actor[1]-position[0], actor[2]-position[1]
                    distance = math.hypot(dx, dy)
                    relative = math.atan2(math.sin(math.atan2(dy, dx)-angle), math.cos(math.atan2(dy, dx)-angle))
                    if not 0.5 < distance < 10 or not scan.angle_min < relative < scan.angle_max:
                        continue
                    half_angle = math.atan2(0.25, distance)
                    i0 = max(0, int((relative-half_angle-scan.angle_min)/scan.angle_step))
                    i1 = min(len(scan.ranges), int((relative+half_angle-scan.angle_min)/scan.angle_step)+1)
                    rays = [(i, float(scan.ranges[i])) for i in range(i0, i1) if math.isfinite(scan.ranges[i])]
                    ray_index, measured = min(rays, key=lambda item: item[1]) if rays else (-1, math.inf)
                    background = self.geometry.static_range(position, angle+scan.angle_min+ray_index*scan.angle_step)
                    if rays and distance-0.65 <= measured <= distance+0.35 and background > distance+0.8:
                        hits.append({"actor": actor_name, "actor_distance_m": distance, "measured_range_m": measured,
                                     "static_background_range_m": background if math.isfinite(background) else None,
                                     "sim_time_s": stamp(scan.header.stamp), "ray_span": [i0, i1]})
                        break
            time.sleep(0.05)
        self.result.check("rendered_pedestrian_lidar", len(hits) >= 3, sensor_pose=[*position, 0.9, 0, 0, angle],
                          topic=topic, matches=hits, interpretation="actual GPU ray returns ahead of static geometry at observed actor bearings")
        if self.process is None:
            self.remove(model_name)
        p = self.poses[name]
        view = [p[1]-4*math.cos(angle), p[2]-4*math.sin(angle), 1.35, 0, 0.06, angle]
        image = self.camera("pedestrians_close", view, hfov=1.2, width=960, height=540, clip_seconds=4)
        self.result.check("walking_camera", image.get("frame_count", 0) >= 10 and image.get("frame_difference_mean", 0) > 0.5,
                          target_actor=name, **image)

    def snapshots(self, selected_views=None):
        snapshots = []
        views = [view for view in self.geometry.layout["viewpoints"]
                 if selected_views is None or view["name"] in selected_views]
        if selected_views is not None and len(views) != len(set(selected_views)):
            raise ValueError("Requested render view is missing from layout")
        for view in views:
            name = view["name"]
            progress(f"Rendering airport view: {name}")
            snapshots.append(self.camera(name, view["pose"], view.get("hfov", 1.0)))
        required_count = len(selected_views) if selected_views else 4
        self.result.check("airport_camera_views", len(snapshots) >= required_count and all(
            s["appearance"]["passes_known_blank_material_regression"] for s in snapshots), views=snapshots,
            interpretation="automated all-white material regression; manual visual review still required")

    def walking_visual_capture(self):
        names = [r["name"] for r in self.geometry.layout["pedestrian_routes"]]
        self.until(lambda: all(n in self.poses for n in names), 30, "pedestrian pose for camera placement")
        selected = None
        for name in names:
            p = self.poses[name]
            for angle in np.arange(0, 2*math.pi, math.pi/8):
                position = [p[1]-4*math.cos(angle), p[2]-4*math.sin(angle)]
                if self.geometry.clearance_at(position) >= 0.7 and self.geometry.static_range(position, angle) > 6:
                    selected = name, position, float(angle)
                    break
            if selected:
                break
        if selected is None:
            raise RuntimeError("No clear pedestrian camera placement")
        name, position, angle = selected
        image = self.camera("pedestrians_close", [*position, 1.35, 0, 0.06, angle],
                            hfov=1.2, width=960, height=540, clip_seconds=4)
        self.result.check("walking_visual_capture", image.get("frame_count", 0) >= 10
                          and image["appearance"]["passes_known_blank_material_regression"],
                          target_actor=name, **image)

    def finish_observations(self):
        samples = list(self.stats)
        dt_sim, dt_wall = samples[-1][0]-samples[0][0], samples[-1][1]-samples[0][1]
        self.result.data["observed_performance"] = {"sim_duration_s": dt_sim, "wall_duration_s": dt_wall,
                                                     "effective_real_time_factor": dt_sim/max(dt_wall, 1e-9),
                                                     "stats_samples": len(samples)}
        (self.result.directory/"pedestrian_poses.json").write_text(json.dumps(list(self.pose_history))+"\n")

    def close(self):
        if self.process is None:
            for name in list(self.created):
                self.remove(name)
        if self.process is not None:
            from gz.msgs10.server_control_pb2 import ServerControl
            from gz.msgs10.world_control_pb2 import WorldControl
            from gz.msgs10.boolean_pb2 import Boolean
            shutdown = self.result.data.setdefault("shutdown", {"events": []})
            def event(name, **details):
                shutdown["events"].append({"event": name, "wall_monotonic_s": time.monotonic(),
                                           "sim_time_s": self.stats[-1][0] if self.stats else None, **details})
                self.result.save()
            try:
                if self.process.poll() is None:
                    event("pause_requested")
                    ok, response = self.node.request("/world/airport/control", WorldControl(pause=True),
                                                     WorldControl, Boolean, 2000)
                    event("pause_response", delivered=bool(ok), accepted=bool(response.data))
                    # Let the queued render frame finish before destroying
                    # skinned actors and temporary GPU sensor render targets.
                    time.sleep(0.25)
                    settled_sim = self.stats[-1][0] if self.stats else None
                    time.sleep(0.75)
                    event("render_settled", sim_advance_s=(self.stats[-1][0]-settled_sim)
                          if self.stats and settled_sim is not None else None)
                    event("stop_requested")
                    ok, response = self.node.request("/server_control", ServerControl(stop=True),
                                                     ServerControl, Boolean, 2000)
                    event("stop_response", delivered=bool(ok), accepted=bool(response.data))
                self.process.wait(timeout=10)
                event("launch_exited", launch_exit_code=self.process.returncode)
            except Exception:
                event("graceful_shutdown_fallback")
                try:
                    os.killpg(self.process.pid, signal.SIGINT)
                    self.process.wait(timeout=15)
                except (ProcessLookupError, subprocess.TimeoutExpired):
                    if self.process.poll() is None:
                        os.killpg(self.process.pid, signal.SIGTERM)
                        try:
                            self.process.wait(timeout=5)
                        except subprocess.TimeoutExpired:
                            os.killpg(self.process.pid, signal.SIGKILL)
                            self.process.wait(timeout=3)
        if self.log_handle is not None:
            self.log_handle.close()
            self.log_handle = None
        if self.ros_executor is not None:
            self.ros_executor.shutdown(timeout_sec=2.0)
            self.ros_executor = None
        if self.ros_node is not None:
            self.ros_node.destroy_node()
            self.ros_node = None
        if self.ros_context is not None and self.ros_context.ok():
            self.rclpy.shutdown(context=self.ros_context)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--share", type=Path, help="installed omslam_sim share directory; inferred from ament when omitted")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--static-only", action="store_true")
    parser.add_argument("--static-evidence", type=Path,
                        help="reuse passing static checks only when installed world/layout hashes still match")
    parser.add_argument("--render-only", action="store_true", help="capture changed visual assets; reuse unchanged functional evidence")
    parser.add_argument("--functional-evidence", type=Path)
    parser.add_argument("--functional-fingerprint", type=Path)
    parser.add_argument("--views", nargs="+", help="render only named layout viewpoints")
    parser.add_argument("--skip-walking-capture", action="store_true", help="reuse an already approved walking clip")
    parser.add_argument("--density-before", type=Path, help="previous copied package assets for density comparison")
    parser.add_argument("--expected-pedestrians", type=int)
    parser.add_argument("--expected-terminals", nargs="+", default=["T1", "T2"])
    parser.add_argument("--expected-bounds", nargs=4, type=float, metavar=("XMIN", "YMIN", "XMAX", "YMAX"))
    parser.add_argument("--minimum-seats", type=int)
    parser.add_argument("--minimum-wide-corridor", type=float, default=6.4,
                        help="current layout's required main corridor width, in metres")
    parser.add_argument("--attach", action="store_true", help="attach to an already running airport; do not stop it")
    parser.add_argument("--startup-timeout", type=float, default=120)
    args = parser.parse_args()
    if args.share is None:
        from ament_index_python.packages import get_package_share_directory
        args.share = Path(get_package_share_directory("omslam_sim"))
    result = Results(args.output)
    if not args.static_only:
        result.data["visual_review"] = {"status": "PENDING", "required": "Inspect rendered furniture, floor detail, colours and signage; automated metrics are not visual approval"}
    simulator = None
    try:
        geometry = Geometry(args.share)
        if args.render_only:
            if not args.functional_evidence or not args.functional_fingerprint:
                raise ValueError("render-only requires functional evidence and its pre-change fingerprint")
            previous = json.loads(args.functional_evidence.read_text())
            expected = json.loads(args.functional_fingerprint.read_text())
            current = functional_fingerprint(args.share)
            if expected != current:
                raise RuntimeError("Functional inputs changed; render-only reuse is not valid")
            keys = ("world_structure", "terminal_connectivity", "pedestrian_route_clearance", "gazebo_time",
                    "ros_clock_bridge", "pedestrian_motion_and_proxy", "rendered_pedestrian_lidar")
            if any(previous["checks"][key]["status"] != "PASS" for key in keys):
                raise RuntimeError("Functional evidence contains a failed check")
            result.data["checks"].update({k: previous["checks"][k] | {"reused_from": str(args.functional_evidence.resolve())} for k in keys})
            result.data["functional_reuse_fingerprint"] = current
            result.data["manifest"] = previous["manifest"] | {
                "world_sha256": digest(args.share/"worlds/airport.sdf"),
                "previous_functional_world_sha256": previous["manifest"]["world_sha256"]}
            result.data["visual_review"] = {"status": "PENDING", "required": "View rendered furniture, floor detail, colours and signage"}
            result.save()
            progress("Reusing seven functional checks: collision/layout/plugin/actor/clock fingerprints unchanged")
        elif args.static_evidence:
            old = json.loads(args.static_evidence.read_text())
            manifest = old["manifest"]
            if manifest["world_sha256"] != digest(args.share/"worlds/airport.sdf") or \
                    manifest["layout_sha256"] != digest(geometry.layout_path):
                raise RuntimeError("Static evidence does not match the installed world/layout")
            if manifest.get("reference_map_sha256") and manifest["reference_map_sha256"] != digest(args.share/"maps/airport_reference.pgm"):
                raise RuntimeError("Static evidence does not match the installed reference map")
            keys = ("world_structure", "terminal_connectivity", "pedestrian_route_clearance")
            if any(old["checks"][k]["status"] != "PASS" for k in keys):
                raise RuntimeError("Static evidence is not passing")
            result.data["checks"].update({k: old["checks"][k] | {"reused_from": str(args.static_evidence.resolve())} for k in keys})
            if old["checks"].get("density_improvement", {}).get("status") == "PASS":
                result.data["checks"]["density_improvement"] = old["checks"]["density_improvement"] | {"reused_from": str(args.static_evidence.resolve())}
            result.data["manifest"] = manifest
            result.save()
            progress("Reusing passing static evidence: installed world/layout/reference hashes unchanged")
        else:
            geometry.static_checks(result, args.minimum_wide_corridor, args.expected_terminals,
                                   args.expected_pedestrians, args.expected_bounds)
        if args.density_before:
            geometry.density_comparison(args.density_before, result, args.expected_pedestrians, args.minimum_seats)
        if any(c["status"] == "FAIL" for c in result.data["checks"].values()):
            raise RuntimeError("Static world checks failed; runtime test was not started")
        if not args.static_only:
            simulator = Simulator(geometry, result, attach=args.attach, startup_timeout=args.startup_timeout,
                                  render_only=args.render_only)
            if args.render_only:
                if not args.skip_walking_capture:
                    simulator.walking_visual_capture()
            else:
                simulator.verify_motion()
                simulator.sensor_people()
            simulator.snapshots(args.views)
            simulator.finish_observations()
    except Exception as exc:
        result.data["error"] = {"type": type(exc).__name__, "message": str(exc), "traceback": traceback.format_exc()}
        progress(traceback.format_exc())
    finally:
        if simulator is not None:
            simulator.close()
    log_path = result.directory/"launch.log"
    if log_path.exists():
        critical = [line for line in log_path.read_text(errors="replace").splitlines() if re.search(
            r"\[Err\]|\[ERROR\]|Segmentation fault|process has died|Failed to load (?:system )?plugin|Unable to load|Unable to find uri|Error Code", line)]
        result.check("gazebo_launch_errors", not critical, messages=critical[:30])
        material_warnings = [line for line in log_path.read_text(errors="replace").splitlines() if re.search(
            r"Missing material|(?:Failed|Unable).*(?:material|texture)", line, re.IGNORECASE)]
        result.check("render_material_warnings", not material_warnings, messages=material_warnings[:30])
    passed = "error" not in result.data and all(c["status"] == "PASS" for c in result.data["checks"].values())
    result.data["status"] = ("PASS" if args.static_only else "AUTOMATED_PASS_VISUAL_REVIEW_REQUIRED") if passed else "FAIL"
    result.save()
    progress(f"{result.data['status']}: {result.directory/'results.json'}")
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
