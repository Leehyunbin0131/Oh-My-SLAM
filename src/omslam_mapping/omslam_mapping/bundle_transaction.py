"""Publish a flat map bundle without exposing a loadable, incomplete YAML."""
from contextlib import contextmanager
import hashlib
import fcntl
import json
import os
from pathlib import Path
import shutil
import tempfile


BUNDLE_SUFFIXES = ('.yaml', '.pgm', '.posegraph', '.data', '.metadata.json')
MAP_FILES = BUNDLE_SUFFIXES + ('.keepout.json',)


def _file(prefix, suffix):
    return prefix.parent / (prefix.name + suffix)


def _same_inode(source, destination):
    try:
        return os.path.samestat(source.lstat(), destination.lstat())
    except FileNotFoundError:
        return False


def _rollback(stage, prefix):
    # Only the hard links created by this attempt may be removed.
    for suffix in MAP_FILES:
        final = _file(prefix, suffix)
        if _same_inode(_file(stage, suffix), final):
            final.unlink()


def _reservation_path(prefix):
    return prefix.parent / ('.' + prefix.name + '.saving')


def check_graph_published(prefix):
    if _reservation_path(prefix).exists() and not _file(prefix, '.yaml').exists():
        raise ValueError(f'Map save has not completed: {prefix}')


@contextmanager
def _reserve(prefix):
    # The parent lock covers entry creation/removal only. Long map operations
    # at distinct prefixes can proceed concurrently, also across processes.
    reservation = _reservation_path(prefix)
    parent_fd = os.open(prefix.parent, os.O_DIRECTORY)
    try:
        fcntl.flock(parent_fd, fcntl.LOCK_EX)
        reservation.mkdir(exist_ok=True)
        lock_fd = os.open(reservation, os.O_DIRECTORY)
        try:
            fcntl.flock(lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            os.close(lock_fd)
            raise RuntimeError(f'Another map save is in progress: {prefix}') from exc
        fcntl.flock(parent_fd, fcntl.LOCK_UN)
        try:
            # After a process interruption the kernel drops its lock. If the
            # YAML had not been published, discard its partial public links.
            for previous in reservation.glob('attempt-*'):
                stage = previous / prefix.name
                if not _same_inode(_file(stage, '.yaml'), _file(prefix, '.yaml')):
                    _rollback(stage, prefix)
                shutil.rmtree(previous)
            yield reservation, parent_fd
        finally:
            fcntl.flock(parent_fd, fcntl.LOCK_EX)
            try:
                reservation.rmdir()
            finally:
                os.close(lock_fd)
    finally:
        os.close(parent_fd)


@contextmanager
def staged_map(prefix, suffixes):
    """Yield a private prefix and publish its validated files, with YAML last."""
    prefix = Path(prefix).absolute()
    suffixes = tuple(suffixes)
    if (len(suffixes) != len(set(suffixes)) or not set(suffixes) <= set(MAP_FILES)
            or not {'.yaml', '.pgm', '.metadata.json'} <= set(suffixes)):
        raise ValueError('Invalid map bundle suffixes')
    prefix.parent.mkdir(parents=True, exist_ok=True)
    with _reserve(prefix) as (reservation, parent_fd):
        existing = [str(_file(prefix, ext)) for ext in (*MAP_FILES, '.keepout.pending')
                    if os.path.lexists(_file(prefix, ext))]
        if existing:
            raise ValueError(f'Refusing to overwrite existing map files: {existing}')
        with tempfile.TemporaryDirectory(prefix='attempt-', dir=reservation) as directory:
            stage = Path(directory) / prefix.name
            try:
                yield stage
                meta_path = _file(stage, '.metadata.json')
                metadata = json.loads(meta_path.read_text(encoding='utf-8'))
                if '.keepout.json' in suffixes:
                    contents = _file(stage, '.keepout.json').read_bytes()
                    metadata['files']['keepout.json'] = {
                        'sha256': hashlib.sha256(contents).hexdigest(), 'size_bytes': len(contents)}
                for key, info in metadata['files'].items():
                    if '.' + key in suffixes:
                        info['path'] = str(_file(prefix, '.' + key))
                meta_path.write_text(json.dumps(metadata, indent=2, sort_keys=True) + '\n', encoding='utf-8')
                for suffix in suffixes:
                    source = _file(stage, suffix)
                    if not source.is_file() or source.stat().st_size == 0:
                        raise ValueError(f'Incomplete map bundle: {source}')
                    with source.open('rb') as file:
                        os.fsync(file.fileno())
                for suffix in suffixes:
                    if suffix != '.yaml':
                        os.link(_file(stage, suffix), _file(prefix, suffix))
                os.fsync(parent_fd)
                os.link(_file(stage, '.yaml'), _file(prefix, '.yaml'))
                os.fsync(parent_fd)
            except BaseException:
                _rollback(stage, prefix)
                raise
