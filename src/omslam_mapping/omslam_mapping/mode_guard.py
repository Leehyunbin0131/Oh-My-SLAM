"""Keep the two map->odom authorities exclusive on this host and ROS domain."""

import fcntl
import os
from pathlib import Path
import tempfile


class ModeGuard:
    """Retain an advisory lock until launch shuts down; never unlink its inode."""

    def __init__(self, mode, directory=None, domain=None):
        domain = int((os.environ.get('ROS_DOMAIN_ID') or '0') if domain is None else domain)
        directory = Path(directory or '/tmp')
        self.path = directory / f'omslam-map-{os.getuid()}-domain-{domain}.lock'
        self._file = self.path.open('a+', encoding='utf-8')
        try:
            fcntl.flock(self._file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            self._file.seek(0)
            owner = self._file.read().strip()
            self._file.close()
            raise RuntimeError(
                f'An omslam_mapping mode already owns map->odom in ROS_DOMAIN_ID={domain}: '
                f'{owner}. Stop that launch before starting {mode}.'
            ) from exc
        self._file.seek(0)
        self._file.truncate()
        self._file.write(f'mode={mode} pid={os.getpid()}\n')
        self._file.flush()

    def close(self):
        if not self._file.closed:
            fcntl.flock(self._file.fileno(), fcntl.LOCK_UN)
            self._file.close()
