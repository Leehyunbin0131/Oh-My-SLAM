#!/usr/bin/env python3
"""Build the P4 package graph and record verifiable source and installation hashes."""
import argparse
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile


WORKSPACE = Path(__file__).resolve().parents[1]
REPORTS = WORKSPACE / 'reports/sim-navigation'
DEFAULT_BUILD = REPORTS / 'latest_build.json'
DEFAULT_VALIDATION = REPORTS / 'latest_validation.json'


def navigation_packages():
    result = subprocess.run(['colcon', 'list', '--base-paths', 'src', '--packages-up-to',
                             'omslam_navigation', '--names-only'], cwd=WORKSPACE,
                            check=True, capture_output=True, text=True)
    packages = sorted(result.stdout.split())
    if 'omslam_navigation_contracts' not in packages:
        raise ValueError('P4 contract probe is missing from the navigation package graph')
    return packages


def tree_hashes(tree, packages):
    hashes = {}
    for package in packages:
        directory = WORKSPACE / tree / package
        if not directory.is_dir():
            raise FileNotFoundError(directory)
        for path in sorted(directory.rglob('*')):
            if (path.suffix in ('.pyc', '.pyo') or '__pycache__' in path.parts
                    or any(part.startswith('.') for part in path.relative_to(directory).parts)):
                continue
            if path.is_file():
                hashes[str(path.relative_to(WORKSPACE))] = hashlib.sha256(path.read_bytes()).hexdigest()
    return hashes


def package_source_hashes():
    return tree_hashes('src', navigation_packages())


def write_manifest(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile('w', dir=path.parent, delete=False) as temporary:
        name = temporary.name
        try:
            json.dump(value, temporary, indent=2)
            temporary.write('\n')
            temporary.flush()
            os.replace(name, path)
        finally:
            Path(name).unlink(missing_ok=True)


def verified_build(path):
    if not path.is_file():
        raise FileNotFoundError(f'Missing build manifest {path}. Run tools/build_navigation.py first.')
    data = json.loads(path.read_text())
    packages = navigation_packages()
    if data.get('exit_code') != 0 or data.get('packages') != packages:
        raise ValueError(f'{path} does not describe a successful build of the current navigation packages.')
    for tree, field in [('src', 'source_sha256'), ('install', 'installed_sha256')]:
        expected = data.get(field) or {}
        actual = tree_hashes(tree, packages)
        if expected != actual:
            changed = sorted(name for name in expected.keys() | actual.keys()
                             if expected.get(name) != actual.get(name))
            raise ValueError(f'{tree} files differ from {path} ({len(changed)}): '
                             f'{", ".join(changed[:5])}. Run tools/build_navigation.py again.')
    return data


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, help='New directory for the log and manifest')
    args = parser.parse_args()
    started = datetime.now().astimezone()
    output = (args.output or REPORTS / started.strftime('build-%Y%m%d-%H%M%S-%f')).resolve()
    output.mkdir(parents=True, exist_ok=False)
    packages = navigation_packages()
    before = tree_hashes('src', packages)
    command = ['colcon', 'build', '--base-paths', 'src', '--packages-up-to', 'omslam_navigation',
               '--parallel-workers', '3', '--cmake-args', '-DCMAKE_BUILD_TYPE=Release',
               '-DPython3_EXECUTABLE=/usr/bin/python3']
    env = os.environ.copy()
    for key in ('AMENT_PREFIX_PATH', 'COLCON_PREFIX_PATH', 'CMAKE_PREFIX_PATH', 'PYTHONPATH', 'LD_LIBRARY_PATH'):
        env.pop(key, None)
    env['PATH'] = '/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin'
    env.setdefault('MAKEFLAGS', '-j2')
    env.setdefault('CMAKE_BUILD_PARALLEL_LEVEL', '2')
    with (output/'build.log').open('w') as log:
        print(f'Building navigation packages. Log: {log.name}', flush=True)
        proc = subprocess.run(['bash', '-c', 'source /opt/ros/jazzy/setup.bash && exec "$@"',
                               'navigation-build', *command], cwd=WORKSPACE, env=env, stdout=log,
                              stderr=subprocess.STDOUT)
    code = proc.returncode
    manifest = {'packages': packages, 'command': command, 'cwd': str(WORKSPACE), 'started': started.isoformat(),
                'source_sha256': before, 'log': str(output/'build.log')}
    if code == 0:
        if before == tree_hashes('src', packages):
            manifest['installed_sha256'] = tree_hashes('install', packages)
        else:
            manifest['error'] = 'Package sources changed during the build'
            code = 2
    manifest['exit_code'] = code
    write_manifest(output/'build.json', manifest)
    if code == 0:
        write_manifest(DEFAULT_BUILD, manifest)
    print(f'{output / "build.json"} (exit {code})', flush=True)
    return code


if __name__ == '__main__':
    raise SystemExit(main())
