#!/usr/bin/env python3
"""Bounded V3: real SLAM map/graph save+reload, then AMCL in the static airport.

Motion uses only /odom. /sim/ground_truth is recorded only for assessment with
one fixed initial world-to-map alignment. No geometry reference map is used as
SLAM output. Reuses the P2 runtime startup/cleanup, not its acceptance suite.
"""
from __future__ import annotations

import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import traceback

import numpy as np
from scipy.ndimage import distance_transform_edt
from scipy.spatial import cKDTree

from verify_sim_robot import Results, Runtime, angle_delta, digest, stamp_s, yaw


def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return np.array([a[0]+c*b[0]-s*b[1], a[1]+s*b[0]+c*b[1], angle_delta(a[2]+b[2], 0.)])


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return np.array([-c*a[0]-s*a[1], s*a[0]-c*a[1], -a[2]])


class MappingRun:
    def __init__(self, runtime):
        self.r, self.result = runtime, runtime.result
        self.maps, self.graphs, self.amcl = deque(maxlen=5), deque(maxlen=3), deque(maxlen=500)
        self.mode_process = self.mode_log = None
        self.world_to_map = None
        self.trajectory = []
        self.stage = 'startup'
        self.prefix = self.result.directory/'airport_local'

    def prepare(self):
        from nav_msgs.msg import OccupancyGrid
        from visualization_msgs.msg import MarkerArray
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
        from ament_index_python.packages import get_package_prefix
        mapping_prefix = Path(get_package_prefix('omslam_mapping')).resolve()
        self.result.check('installed_mapping_package', mapping_prefix == self.r.workspace/'install/omslam_mapping', prefix=str(mapping_prefix))
        self.result.data['mapping_installed_sha256'] = {str(p): digest(p) for p in mapping_prefix.rglob('*')
                                                       if p.is_file() and p.suffix in ('.py', '.yaml', '.json')}
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.r.node.create_subscription(OccupancyGrid, '/map', lambda msg: self.maps.append(msg), qos)
        self.r.node.create_subscription(MarkerArray, '/slam_toolbox/graph_visualization', lambda msg: self.graphs.append(msg), 1)
        self.r.node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', lambda msg: self.amcl.append(msg), 20)
        self.initial_pub = self.r.node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
        self.Pose = PoseWithCovarianceStamped
        self.r.node.create_timer(.2, self.record)

    def map_pose(self):
        from rclpy.time import Time
        tf = self.r.buffer.lookup_transform('map', 'base_footprint', Time()).transform
        return np.array([tf.translation.x, tf.translation.y, yaw(tf.rotation)])

    def record(self):
        if self.world_to_map is None:
            return
        try:
            estimate = self.map_pose()
            truth = compose(self.world_to_map, self.r.pose())
            self.trajectory.append({'time': self.r.sim_time, 'stage': self.stage, 'odom': self.r.pose('/odom').tolist(),
                                    'truth': truth.tolist(), 'estimate': estimate.tolist()})
        except Exception:
            pass

    def start_mode(self, mode, label, *arguments):
        if self.mode_process is not None:
            raise RuntimeError('Stop prior mode before changing map->odom owner')
        self.stage = label
        self.mode_log = (self.result.directory/(label+'.log')).open('w')
        command = ['ros2', 'launch', 'omslam_mapping', mode+'.launch.py', 'use_sim_time:=true', *arguments]
        self.mode_process = subprocess.Popen(command, cwd='/tmp', env=os.environ.copy(),
            stdout=self.mode_log, stderr=subprocess.STDOUT, start_new_session=True)
        self.result.data.setdefault('mode_commands', []).append({'label': label, 'command': command, 'pid': self.mode_process.pid})

    def stop_mode(self):
        if self.mode_process is None:
            return
        process = self.mode_process
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=12)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
        self.result.data.setdefault('mode_shutdowns', []).append({'pid': process.pid, 'returncode': process.returncode})
        self.mode_process = None
        self.mode_log.close()
        self.mode_log = None
        self.r.until(lambda: not any(p.node_name in ('slam_toolbox', 'amcl') for p in self.r.node.get_publishers_info_by_topic('/tf')),
                     8., 'old map transform authority to leave ROS graph')

    def cli(self, label, *arguments):
        path = self.result.directory/(label+'.log')
        with path.open('w') as handle:
            command = ['ros2', 'run', 'omslam_mapping', *arguments]
            process = subprocess.Popen(command, stdout=handle, stderr=subprocess.STDOUT, env=os.environ.copy(), start_new_session=True)
            try:
                self.r.until(lambda: process.poll() is not None, 35., label+' completion')
            finally:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                    process.wait(timeout=5.)
        self.result.data.setdefault('cli_commands', []).append({'command': command, 'exit_code': process.returncode, 'log': str(path)})
        if process.returncode:
            raise RuntimeError(label+' failed: '+path.read_text()[-4000:])

    def goto(self, target):
        deadline = time.monotonic()+50.
        while True:
            pose = self.r.pose('/odom')
            delta = np.asarray(target)-pose[:2]
            distance = float(np.linalg.norm(delta))
            if distance < .035:
                break
            if time.monotonic() > deadline:
                raise TimeoutError('Odom-feedback waypoint '+str(target))
            error = angle_delta(math.atan2(delta[1], delta[0]), pose[2])
            vx = min(.22, .8*distance) if abs(error) < .3 else 0.
            self.r.command(vx, float(np.clip(1.6*error, -.55, .55)))
            self.r.pump(.035)
        self.r.command()
        self.r.wait_sim(.3)

    def face(self, heading):
        deadline = time.monotonic()+20.
        while abs(angle_delta(heading, self.r.pose('/odom')[2])) > .025:
            if time.monotonic() > deadline:
                raise TimeoutError('Odom-feedback final heading')
            error = angle_delta(heading, self.r.pose('/odom')[2])
            self.r.command(0., float(np.clip(1.6*error, -.5, .5)))
            self.r.pump(.035)
        self.r.command()
        self.r.wait_sim(.8)

    def authority(self, expected, label):
        endpoints = self.r.node.get_publishers_info_by_topic('/tf')
        owners = sorted(set(p.node_name for p in endpoints))
        active_backends = [name for name in self.r.node.get_node_names() if name in ('slam_toolbox', 'amcl')]
        edge = self.r.tf_edges.get(('map', 'odom'))
        stamp = stamp_s(edge[-1].header.stamp) if edge else 0.
        clocks = {}
        for name in ([expected, 'map_server'] if expected == 'amcl' else [expected]):
            client = self.r.node.create_client(self.r.GetParameters, '/'+name+'/get_parameters')
            self.r.until(client.service_is_ready, 3., name+' parameters')
            response = self.r.call(client, self.r.GetParameters.Request(names=['use_sim_time']))
            clocks[name] = response.values[0].bool_value
            self.r.node.destroy_client(client)
        self.result.check(label, owners == sorted(['robot_state_publisher', 'ekf_filter_node', expected])
                          and active_backends == [expected]
                          and 0 < stamp < self.r.sim_time+1.5 and self.r.sim_time-stamp < 2. and all(clocks.values()),
                          tf_publishers=owners, map_odom_stamp_s=stamp, clock_s=self.r.sim_time,
                          active_localization_backends=active_backends,
                          tf_endpoint_counts={name: sum(p.node_name == name for p in endpoints) for name in owners},
                          use_sim_time=clocks,
                          evidence='One active map backend, its fresh map->odom edge and TF configuration; duplicate endpoints from the same node are not separate authorities')

    def graph(self):
        nodes, segments = {}, []
        for marker in self.graphs[-1].markers:
            if marker.ns == 'slam_toolbox':
                nodes[marker.id] = [marker.pose.position.x, marker.pose.position.y]
            elif marker.ns == 'slam_toolbox_edges':
                points = marker.points
                segments += [([a.x, a.y], [b.x, b.y]) for a, b in zip(points[::2], points[1::2])]
        if not nodes:
            raise AssertionError('No graph nodes observed')
        ids = sorted(nodes)
        tree = cKDTree([nodes[i] for i in ids])
        edges = []
        for a, b in segments:
            distance, index = tree.query([a, b])
            if max(distance) < 1e-4:
                edges.append([ids[int(index[0])], ids[int(index[1])]])
        return nodes, edges

    def alignment(self, label):
        grid = self.maps[-1]
        cells = np.asarray(grid.data).reshape(grid.info.height, grid.info.width)
        dist = distance_transform_edt(cells < 65)*grid.info.resolution
        scan = self.r.samples['/scan'][-1][1]
        from rclpy.time import Time
        tf = self.r.buffer.lookup_transform('map', scan.header.frame_id, Time()).transform
        heading = yaw(tf.rotation)
        errors = []
        for i, value in enumerate(scan.ranges):
            if not math.isfinite(value) or not scan.range_min <= value < scan.range_max-.1:
                continue
            angle = heading+scan.angle_min+i*scan.angle_increment
            x, y = tf.translation.x+value*math.cos(angle), tf.translation.y+value*math.sin(angle)
            col = int((x-grid.info.origin.position.x)/grid.info.resolution)
            row = int((y-grid.info.origin.position.y)/grid.info.resolution)
            if 0 <= row < dist.shape[0] and 0 <= col < dist.shape[1]:
                errors.append(dist[row, col])
        metrics = {'points': len(errors), 'median_m': float(np.median(errors)) if errors else None,
                   'p95_m': float(np.percentile(errors, 95)) if errors else None,
                   'fraction_within_015m': float(np.mean(np.asarray(errors) <= .15)) if errors else 0.}
        self.result.check(label, len(errors) > 100 and metrics['median_m'] <= .08 and metrics['p95_m'] <= .20
                          and metrics['fraction_within_015m'] >= .80, **metrics)

    def error(self, label):
        truth, estimate = compose(self.world_to_map, self.r.pose()), self.map_pose()
        position = float(np.linalg.norm(estimate[:2]-truth[:2])); heading = abs(angle_delta(estimate[2], truth[2]))
        self.result.check(label, position <= .15 and heading <= .20, position_error_m=position,
                          yaw_error_rad=heading, estimate_map=estimate.tolist(), truth_map=truth.tolist(),
                          fixed_world_to_map=self.world_to_map.tolist())
        return estimate

    def run(self):
        self.prepare()
        self.start_mode('mapping', 'mapping')
        self.r.until(lambda: self.maps and self.graphs and ('map', 'odom') in self.r.tf_edges, 25., 'SLAM occupancy map, graph and transform')
        self.r.wait_sim(1.)
        origin = self.r.pose('/odom').copy()
        self.world_to_map = compose(self.map_pose(), inverse(self.r.pose()))
        self.result.data['fixed_world_to_map'] = self.world_to_map.tolist()
        for point in ([3., 0.], [3., 2.], [0., 2.], [0., 0.]):
            goal = compose(origin, np.array([*point, 0.]))
            print('Driving odom waypoint '+str(goal[:2].tolist()), flush=True)
            self.goto(goal[:2])
        self.face(origin[2])
        self.r.wait_sim(2.)
        self.result.data['closed_loop_odom'] = {
            'start': origin.tolist(), 'finish': self.r.pose('/odom').tolist(),
            'waypoints_relative_start': [[3., 0.], [3., 2.], [0., 2.], [0., 0.]],
            'waypoint_position_tolerance_m': .035}
        self.cli('save', 'save_map', '--output', str(self.prefix), '--backend', 'simulation', '--workspace', str(self.r.workspace))
        self.authority('slam_toolbox', 'mapping_single_map_tf')
        nodes, edges = self.graph()
        self.saved_graph = {'nodes': nodes, 'edges': edges}
        ids = sorted(nodes)
        cumulative = np.concatenate(([0.], np.cumsum([np.linalg.norm(np.subtract(nodes[b], nodes[a])) for a, b in zip(ids, ids[1:])])))
        path_at = dict(zip(ids, cumulative))
        revisit = [[a, b] for a, b in edges if abs(path_at[a]-path_at[b]) > 6.
                   and min(a, b) <= ids[max(1, len(ids)//4)] and max(a, b) >= ids[3*len(ids)//4]]
        (self.result.directory/'pose_graph.json').write_text(json.dumps({'nodes': nodes, 'edges': edges}, indent=2)+'\n')
        self.result.check('closed_loop_graph_revisit_connections', len(nodes) > 25 and revisit,
                          nodes=len(nodes), edges=len(edges), node_path_length_m=float(cumulative[-1]), early_late_edges=revisit,
                          interpretation='Observed early/late graph edges across the completed closed route. Karto selects by scan barycenter, so a robot-pose distance cutoff is not a valid proxy.')
        self.alignment('mapped_scan_wall_alignment')
        final_slam = self.error('slam_truth_accuracy_after_loop')
        grid = self.maps[-1]
        self.saved_grid = grid
        cells = np.asarray(grid.data)
        self.result.check('actual_slam_map_has_free_and_occupied_cells', np.isclose(grid.info.resolution, .05)
                          and int((cells == 0).sum()) > 2000 and int((cells >= 65).sum()) > 100,
                          dimensions=[grid.info.width, grid.info.height], resolution_m=grid.info.resolution,
                          free_cells=int((cells == 0).sum()), occupied_cells=int((cells >= 65).sum()))
        paths = [Path(str(self.prefix)+suffix) for suffix in ('.yaml', '.pgm', '.posegraph', '.data', '.metadata.json')]
        self.result.check('map_graph_and_metadata_saved', all(p.is_file() and p.stat().st_size > 0 for p in paths),
                          files={str(p): {'bytes': p.stat().st_size, 'sha256': digest(p)} for p in paths if p.is_file()})
        metadata = json.loads(Path(str(self.prefix)+'.metadata.json').read_text())
        self.result.check('saved_metadata_matches_actual_bundle', metadata['backend'] == 'simulation'
                          and metadata['runtime_parameters']['values']['use_sim_time'] is True
                          and metadata['dependencies']['repositories']
                          and metadata['geometry']['model_provenance']['sha256']
                          and all(metadata['files'][suffix]['sha256'] == digest(Path(str(self.prefix)+'.'+suffix))
                                  for suffix in ('yaml', 'pgm', 'posegraph', 'data')),
                          backend=metadata['backend'], source_commit=metadata['source']['commit'],
                          dependencies=metadata['dependencies'], model_provenance=metadata['geometry']['model_provenance'])
        self.stop_mode()
        self.maps.clear(); self.graphs.clear()
        self.start_mode('mapping', 'graph_reload')
        self.r.until(lambda: self.maps and self.graphs, 20., 'fresh mapper before graph reload')
        self.cli('load_graph', 'load_graph', '--input', str(self.prefix), '--match', 'pose',
                 '--x', str(final_slam[0]), '--y', str(final_slam[1]), '--yaw', str(final_slam[2]))
        self.r.wait_sim(2.)
        loaded_nodes, loaded_edges = self.graph()
        self.result.check('serialized_graph_restored_in_new_mapper', len(loaded_nodes) >= len(nodes),
                          nodes_before=len(nodes), nodes_after=len(loaded_nodes), edges_before=len(edges), edges_after=len(loaded_edges))
        self.alignment('reloaded_graph_scan_alignment')
        self.localize(final_slam)

    def localize(self, final_slam):
        self.stop_mode()
        self.maps.clear(); self.amcl.clear()
        self.start_mode('localization', 'amcl', 'map:='+str(self.prefix)+'.yaml')
        self.r.until(lambda: self.maps and self.initial_pub.get_subscription_count() > 0, 25., 'AMCL loaded map and initial pose subscriber')
        if not hasattr(self, 'saved_grid'):
            self.saved_grid = self.maps[-1]
        state_client = self.r.node.create_client(self.r.GetState, '/amcl/get_state')
        self.r.until(state_client.service_is_ready, 5., 'AMCL lifecycle state service')
        self.r.until(lambda: self.r.call(state_client, self.r.GetState.Request()).current_state.label == 'active',
                     10., 'AMCL active before initial pose')
        self.r.node.destroy_client(state_client)
        seed = final_slam + np.array([.35, -.25, .25])
        msg = self.Pose(); msg.header.frame_id = 'map'; msg.header.stamp = self.r.node.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = float(seed[0]), float(seed[1])
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(seed[2]/2), math.cos(seed[2]/2)
        msg.pose.covariance[0] = msg.pose.covariance[7] = .25**2; msg.pose.covariance[35] = .20**2
        self.initial_pub.publish(msg)
        self.seed = seed
        true_seed = compose(self.world_to_map, self.r.pose())
        self.result.data['amcl_initial_seed'] = {'pose_map': seed.tolist(), 'source': 'saved SLAM estimated pose plus deliberate offset; no ground truth input',
                                              'position_error_m': float(np.linalg.norm(seed[:2]-true_seed[:2])),
                                              'yaw_error_rad': abs(angle_delta(seed[2], true_seed[2]))}
        self.r.until(lambda: self.amcl, 15., 'AMCL accepted initial pose')
        local = self.r.pose('/odom').copy()
        observation_waypoints = ((2., 0.), (2., 1.))
        self.result.data['amcl_observation_waypoints_relative_odom'] = observation_waypoints
        for point in observation_waypoints:
            self.goto(compose(local, np.array([*point, 0.]))[:2])
        self.face(local[2]+.35)
        self.face(local[2])
        self.r.wait_sim(2.)
        self.result.data['amcl_pose_update_count'] = len(self.amcl)
        self.authority('amcl', 'amcl_single_map_tf_after_slam_exit')
        self.amcl_final = self.error('amcl_converges_from_offset_initial_pose')
        self.alignment('amcl_scan_alignment_to_saved_map')
        self.plot()

    def plot(self):
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        grid = self.saved_grid
        cells = np.asarray(grid.data).reshape(grid.info.height, grid.info.width)
        image = np.where(cells < 0, .65, 1.-cells/100.)
        x, y, resolution = grid.info.origin.position.x, grid.info.origin.position.y, grid.info.resolution
        fig, ax = plt.subplots(figsize=(10, 8))
        ax.imshow(image, origin='lower', cmap='gray', vmin=0, vmax=1,
                  extent=(x, x+grid.info.width*resolution, y, y+grid.info.height*resolution))
        graph = getattr(self, 'saved_graph', None)
        if graph:
            nodes = graph['nodes']
            points = np.asarray([nodes[k] for k in sorted(nodes, key=int)])
            ax.plot(points[:, 0], points[:, 1], '.-', color='#1670c5', linewidth=1., markersize=3,
                    label='Saved SLAM graph nodes')
        for stage, key, label, color in [('mapping', 'truth', 'Mapping truth (recorded)', '#a96617'),
                                       ('amcl', 'estimate', 'AMCL estimate', '#158348'),
                                       ('amcl', 'truth', 'AMCL truth (fixed alignment)', '#d95721')]:
            data = [p for p in self.trajectory if p['stage'] == stage]
            if data:
                points = np.asarray([p[key] for p in data])
                ax.plot(points[:, 0], points[:, 1], label=label, color=color, linewidth=1.7)
        ax.scatter(*self.seed[:2], marker='x', color='#bb2990', label='AMCL offset seed', s=70)
        ax.scatter(*self.amcl_final[:2], marker='o', color='#158348', label='AMCL converged', s=45)
        ax.set(xlabel='Map x [m]', ylabel='Map y [m]', title='Actual slam_toolbox map: local airport loop and AMCL restart')
        ax.legend(loc='upper right'); ax.set_aspect('equal'); fig.tight_layout()
        fig.savefig(self.result.directory/'map_with_trajectory.png', dpi=160); plt.close(fig)
        (self.result.directory/'trajectory.json').write_text(json.dumps(self.trajectory, indent=2)+'\n')
        self.result.data['visual_review'] = {'status': 'PENDING', 'map_overlay': str(self.result.directory/'map_with_trajectory.png'),
                                              'required': 'Inspect walls for gross doubling and the closed trajectory in the actual SLAM map'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--timeout', type=float, default=420.)
    args = parser.parse_args()
    os.environ['ROS_DOMAIN_ID'] = '175'; os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    os.environ.pop('ROS_STATIC_PEERS', None)
    os.environ['GZ_PARTITION'] = 'omslam_mapping_verify_'+str(os.getpid())
    os.environ['OMSLAM_GZ_PARTITION'] = os.environ['GZ_PARTITION']
    result = Results(args.output)
    result.data.update(scope='V3 actual Gazebo LiDAR SLAM map/posegraph reload and AMCL restart',
                       unverified=['physical hardware and real mapping', 'Nav2', 'complete airport mapping'])
    runtime = Runtime(result, Path(__file__).resolve().parents[1], args.timeout, pedestrians=False)
    mapping = MappingRun(runtime)
    try:
        runtime.initialize(); mapping.run()
    except Exception as exc:
        result.data['error'] = {'type': type(exc).__name__, 'message': str(exc), 'traceback': traceback.format_exc()}
        print(traceback.format_exc(), flush=True)
    finally:
        (result.directory/'trajectory.json').write_text(json.dumps(mapping.trajectory, indent=2)+'\n')
        try: mapping.stop_mode()
        except Exception as exc: result.data['mode_cleanup_error'] = str(exc)
        runtime.snapshot(); runtime.close()
    launch_log = result.directory/'launch.log'
    if launch_log.exists():
        result.data['known_gazebo_teardown_errors'] = [line for line in launch_log.read_text(errors='replace').splitlines()
                                                       if 'Segmentation fault' in line or 'exit code 139' in line]
    result.data['status'] = 'AUTOMATED_PASS_VISUAL_REVIEW_REQUIRED' if 'error' not in result.data else 'FAIL'
    result.save(); print(result.data['status']+': '+str(result.directory/'results.json'), flush=True)
    return 0 if 'error' not in result.data else 1


if __name__ == '__main__':
    raise SystemExit(main())
