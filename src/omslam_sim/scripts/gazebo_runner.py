#!/usr/bin/env python3
"""Shut down only this Gazebo process tree, requesting transport shutdown first."""
import os
import argparse
from pathlib import Path
import signal
import subprocess
import sys
import time


def descendants(pid):
    try:
        children = Path(f'/proc/{pid}/task/{pid}/children').read_text().split()
    except (FileNotFoundError, ProcessLookupError):
        return []
    result = []
    for child in children:
        result.extend(descendants(int(child)))
        result.append(int(child))
    return result


def identity(pid):
    try:
        # Field 22 is the process start tick. Preserve PID identity across reuse.
        return Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()[19]
    except (FileNotFoundError, ProcessLookupError, IndexError):
        return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--world', required=True, choices=['airport', 'p4_course'])
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    if not args.command:
        parser.error('a Gazebo command is required')
    stopping = False

    def request_stop(_signum, _frame):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    process = subprocess.Popen(args.command, start_new_session=True)
    owned = {}

    def remember_tree():
        for pid in descendants(process.pid) + [process.pid]:
            token = identity(pid)
            if token is not None:
                owned[pid] = token

    try:
        while process.poll() is None:
            remember_tree()
            if stopping:
                try:
                    # Let active camera render jobs settle before destroying
                    # the selected world's sensor/rendering resources.
                    subprocess.run(
                        ['gz', 'service', '-s', f'/world/{args.world}/control',
                         '--reqtype', 'gz.msgs.WorldControl',
                         '--reptype', 'gz.msgs.Boolean', '--timeout', '2000',
                         '--req', 'pause: true'],
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                        timeout=3, check=False,
                    )
                    time.sleep(1.)
                    subprocess.run(
                        ['gz', 'service', '-s', '/server_control',
                         '--reqtype', 'gz.msgs.ServerControl',
                         '--reptype', 'gz.msgs.Boolean', '--timeout', '2000',
                         '--req', 'stop: true'],
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                        timeout=3, check=False,
                    )
                    process.wait(timeout=5)
                except (subprocess.TimeoutExpired, OSError):
                    pass
                break
            time.sleep(0.1)
    finally:
        remember_tree()
        for sig in (signal.SIGTERM, signal.SIGKILL):
            for pid, token in owned.items():
                if identity(pid) != token:
                    continue
                try:
                    os.kill(pid, sig)
                except ProcessLookupError:
                    pass
            try:
                process.wait(timeout=1)
            except subprocess.TimeoutExpired:
                pass
    code = process.returncode
    if not stopping and code:
        raise SystemExit(code if code > 0 else 128 - code)


if __name__ == '__main__':
    main()
