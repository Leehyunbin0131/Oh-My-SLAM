#!/usr/bin/env python3
"""Record a bounded incremental build of the operator's workspace dependency graph."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
from datetime import datetime

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
parser.add_argument('--output', required=True)
args = parser.parse_args()
out = (ROOT / args.output).resolve()
out.mkdir(parents=True, exist_ok=False)

def sources():
    paths = [ROOT/'console.sh', *sorted((ROOT/'src').rglob('*'))]
    return {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in paths if path.is_file()
            and not any(part.startswith('.') or part == '__pycache__' for part in path.relative_to(ROOT).parts)
            and path.suffix not in ('.pyc', '.pyo')}

env = os.environ.copy()
for name in ('AMENT_PREFIX_PATH', 'COLCON_PREFIX_PATH', 'CMAKE_PREFIX_PATH', 'PYTHONPATH', 'LD_LIBRARY_PATH'):
    env.pop(name, None)
env['PATH'] = '/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin'
command = ['colcon', 'build', '--base-paths', 'src', '--packages-up-to', 'omslam_operator_support', '--parallel-workers', '3',
           '--cmake-args', '-DCMAKE_BUILD_TYPE=Release', '-DPython3_EXECUTABLE=/usr/bin/python3']
manifest = {'started_at': datetime.now().astimezone().isoformat(), 'cwd': str(ROOT),
            'command': command, 'source_sha256': sources(), 'log': str(out / 'build.log')}
with (out / 'build.log').open('w') as log:
    result = subprocess.run(['bash', '-c', 'source /opt/ros/jazzy/setup.bash && exec "$@"', 'operator-build', *command],
                            cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT)
manifest.update(exit_code=result.returncode, completed_at=datetime.now().astimezone().isoformat(),
                source_unchanged=manifest['source_sha256'] == sources())
text = json.dumps(manifest, indent=2) + '\n'
(out / 'build.json').write_text(text)
(ROOT / 'reports/operator').mkdir(parents=True, exist_ok=True)
(ROOT / 'reports/operator/latest_build.json').write_text(text)
print(json.dumps({key: manifest[key] for key in ('exit_code', 'source_unchanged', 'log')}), flush=True)
raise SystemExit(result.returncode if result.returncode else (0 if manifest['source_unchanged'] else 2))
