"""Save occupancy/pose-graph files with provenance, or load an existing graph."""

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

import yaml

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.parameter import parameter_value_to_python
from rclpy.parameter_client import AsyncParameterClient
from slam_toolbox.srv import DeserializePoseGraph, SerializePoseGraph

from .bundle_transaction import BUNDLE_SUFFIXES, check_graph_published, staged_map


def _file_info(path):
    path = Path(path).resolve(strict=True)
    return {'path': str(path), 'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
            'size_bytes': path.stat().st_size}


def _json_hash(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':')).encode()).hexdigest()


def _prefix(value):
    path = Path(value).expanduser()
    if not path.is_absolute():
        raise ValueError('Use an absolute map prefix, without a filename extension')
    if any(str(path).endswith(suffix) for suffix in BUNDLE_SUFFIXES):
        raise ValueError('Use a map prefix without .yaml/.pgm/.posegraph/.data/.metadata.json')
    return path.resolve()


def _await(node, future, timeout):
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done():
        raise RuntimeError(f'Service response timed out after {timeout:g} seconds')
    if future.exception():
        raise RuntimeError(str(future.exception()))
    response = future.result()
    if response is None:
        raise RuntimeError('Service returned no response')
    return response


def _call(node, service_type, name, request, timeout):
    client = node.create_client(service_type, name)
    try:
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f'Service unavailable: {name}; start mapping.launch.py first')
        return _await(node, client.call_async(request), timeout)
    finally:
        node.destroy_client(client)


def _parameters(node, timeout):
    client = AsyncParameterClient(node, '/slam_toolbox')
    if not client.wait_for_services(timeout_sec=timeout):
        raise RuntimeError('slam_toolbox parameter services are unavailable')
    names = sorted(_await(node, client.list_parameters(), timeout).result.names)
    values = _await(node, client.get_parameters(names), timeout).values
    return {name: parameter_value_to_python(value) for name, value in zip(names, values)}


def _git(workspace):
    workspace = Path(workspace).expanduser().resolve(strict=True)

    def run(*args):
        return subprocess.run(['git', '-C', str(workspace), *args], check=True,
                              stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=10).stdout

    return {
        'workspace': str(workspace),
        'commit': run('rev-parse', 'HEAD').decode().strip(),
        'dirty': bool(run('status', '--porcelain', '--untracked-files=normal').strip()),
        'tracked_diff_sha256': hashlib.sha256(run('diff', '--binary', 'HEAD')).hexdigest(),
    }


def _package_version(package):
    manifest = Path(get_package_share_directory(package)) / 'package.xml'
    return {'version': ET.parse(manifest).getroot().findtext('version'),
            'manifest': _file_info(manifest)}


def _save(args):
    prefix = _prefix(args.output)
    with staged_map(prefix, BUNDLE_SUFFIXES) as stage:
        _write_bundle(args, stage)
    print(json.dumps({'saved': str(prefix), 'metadata': str(prefix) + '.metadata.json'}, sort_keys=True))


def _write_bundle(args, prefix):
    # Validate provenance before invoking either service that writes files.
    source = _git(args.workspace)
    dependency_file = Path(source['workspace']) / 'dependencies.repos'
    dependencies = {
        'manifest': _file_info(dependency_file),
        'repositories': yaml.safe_load(dependency_file.read_text(encoding='utf-8'))['repositories'],
    }
    share = Path(get_package_share_directory('omslam_mapping'))
    geometry = Path(args.geometry_config) if args.geometry_config else (
        Path(get_package_share_directory('omslam_sim')) / 'config' / 'sim_base.yaml')
    calibration = Path(args.calibration_config) if args.calibration_config else (
        Path(get_package_share_directory('omslam_localization')) / 'config' / 'imu.yaml')
    configurations = {name: _file_info(share / 'config' / f'{name}.yaml')
                      for name in ('mapping', 'localization')}
    geometry_info, calibration_info = _file_info(geometry), _file_info(calibration)
    provenance_file = Path(get_package_share_directory('omslam_description')) / 'provenance.json'
    provenance = json.loads(provenance_file.read_text(encoding='utf-8'))
    geometry_info['model_provenance'] = {
        **_file_info(provenance_file),
        'repository': provenance['repository'], 'commit': provenance['commit'],
    }
    prefix.parent.mkdir(parents=True, exist_ok=True)
    node = rclpy.create_node('omslam_save_map')
    try:
        parameters = _parameters(node, args.timeout)
        if parameters.get('mode') != 'mapping':
            raise RuntimeError('Map bundles must be saved from the mapping backend')
        # Keep SLAM's executor available while the saver discovers/subscribes to
        # /map. The upstream SaveMap callback blocks on this CLI with a fixed
        # two-second subscription timeout, ignoring this tool's timeout setting.
        raster = subprocess.run([
            'ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', str(prefix),
            '--ros-args', '-p', f'save_map_timeout:={float(args.timeout)}',
            '-p', 'map_subscribe_transient_local:=true',
        ], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
            timeout=args.timeout + 5.)
        if raster.returncode:
            raise RuntimeError(f'Nav2 map save failed: {raster.stdout.strip()}')
        yaml_path = Path(str(prefix) + '.yaml')
        config = yaml.safe_load(yaml_path.read_text(encoding='utf-8'))
        image_path = Path(config['image'])
        if (prefix.parent / image_path).resolve() != Path(str(prefix) + '.pgm'):
            raise ValueError('Map saver referenced an unexpected image')
        if image_path.is_absolute():
            config['image'] = image_path.name
            yaml_path.write_text(yaml.safe_dump(config, sort_keys=False), encoding='utf-8')
        graph_request = SerializePoseGraph.Request()
        graph_request.filename = str(prefix)
        graph_response = _call(node, SerializePoseGraph, '/slam_toolbox/serialize_map',
                               graph_request, args.timeout)
        if graph_response.result != SerializePoseGraph.Response.RESULT_SUCCESS:
            raise RuntimeError(f'Pose graph save failed with result={graph_response.result}')
        files = {suffix[1:]: _file_info(str(prefix) + suffix) for suffix in BUNDLE_SUFFIXES[:-1]}
        if any(info['size_bytes'] == 0 for info in files.values()):
            raise RuntimeError('Map service produced an empty bundle file')
        metadata = {
            'schema_version': 1,
            'saved_at_utc': datetime.now(timezone.utc).isoformat(),
            'backend': args.backend,
            'algorithm': 'slam_toolbox_async',
            'source': source,
            'dependencies': dependencies,
            'frames': {key: parameters.get(key) for key in ('map_frame', 'odom_frame', 'base_frame')},
            'files': files,
            'package_default_configurations': configurations,
            'runtime_parameters': {'node': '/slam_toolbox', 'values': parameters,
                                   'sha256': _json_hash(parameters)},
            'geometry': geometry_info,
            'calibration': {
                **calibration_info,
                'status': ('supplied_configuration' if args.calibration_config else
                           'simulation_assumptions_not_physical_calibration'),
            },
            'packages': {name: _package_version(name)
                         for name in ('omslam_mapping', 'slam_toolbox', 'nav2_amcl', 'nav2_map_server')},
        }
        destination = Path(str(prefix) + '.metadata.json')
        with destination.open('x', encoding='utf-8') as output:
            json.dump(metadata, output, indent=2, sort_keys=True)
            output.write('\n')
    finally:
        node.destroy_node()


def _load(args):
    prefix = _prefix(args.input)
    for suffix in ('.posegraph', '.data'):
        if not Path(str(prefix) + suffix).is_file():
            raise ValueError(f'Missing pose graph file: {prefix}{suffix}')
    check_graph_published(prefix)
    metadata_path = Path(str(prefix) + '.metadata.json')
    if metadata_path.is_file():
        metadata = json.loads(metadata_path.read_text(encoding='utf-8'))
        for suffix in ('posegraph', 'data'):
            actual = _file_info(f'{prefix}.{suffix}')['sha256']
            if actual != metadata['files'][suffix]['sha256']:
                raise ValueError(f'Pose graph hash does not match metadata: {prefix}.{suffix}')
    node = rclpy.create_node('omslam_load_graph')
    try:
        request = DeserializePoseGraph.Request()
        request.filename = str(prefix)
        request.match_type = (DeserializePoseGraph.Request.START_AT_FIRST_NODE if args.match == 'first'
                              else DeserializePoseGraph.Request.START_AT_GIVEN_POSE)
        request.initial_pose.x = args.x
        request.initial_pose.y = args.y
        request.initial_pose.theta = args.yaw
        _call(node, DeserializePoseGraph, '/slam_toolbox/deserialize_map', request, args.timeout)
        # Jazzy's response is empty: only subsequent map/graph data establish successful loading.
        print(json.dumps({'deserialize_service_completed': str(prefix), 'match': args.match,
                          'verification': 'Observe /map and /slam_toolbox/graph_visualization'},
                         sort_keys=True))
    finally:
        node.destroy_node()


def _run(parser, operation, argv):
    args = parser.parse_args(argv)
    if args.timeout <= 0:
        parser.error('--timeout must be positive')
    rclpy.init(args=[])
    try:
        operation(args)
    except (ValueError, RuntimeError, OSError, subprocess.SubprocessError, KeyError) as exc:
        print(f'{parser.prog}: {exc}', file=sys.stderr)
        return 1
    finally:
        rclpy.shutdown()
    return 0


def save_main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, help='Absolute output prefix without extension')
    parser.add_argument('--backend', choices=['simulation'], default='simulation')
    parser.add_argument('--workspace', default=str(Path.cwd()), help='Source Git workspace')
    parser.add_argument('--geometry-config', help='Geometry configuration actually used for this run')
    parser.add_argument('--calibration-config', help='Calibration configuration actually used for this run')
    parser.add_argument('--timeout', type=float, default=60.0, help='Per-service wall timeout in seconds')
    return _run(parser, _save, argv)


def load_main(argv=None):
    parser = argparse.ArgumentParser(description='Load a serialized graph into running slam_toolbox')
    parser.add_argument('--input', required=True, help='Absolute graph prefix without extension')
    parser.add_argument('--match', choices=['first', 'pose'], default='first')
    parser.add_argument('--x', type=float, default=0.0)
    parser.add_argument('--y', type=float, default=0.0)
    parser.add_argument('--yaw', type=float, default=0.0)
    parser.add_argument('--timeout', type=float, default=60.0)
    return _run(parser, _load, argv)
