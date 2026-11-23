import json
import os
from pathlib import Path
from types import SimpleNamespace

import pytest
from rclpy.executors import ExternalShutdownException
from omslam_mapping import map_bundle
from omslam_mapping.bundle_transaction import BUNDLE_SUFFIXES, staged_map


def test_graph_failure_retries_same_prefix_and_excludes_competing_save(tmp_path, monkeypatch):
    prefix = tmp_path / 'maps' / 'warehouse'
    (tmp_path / 'dependencies.repos').write_text('repositories: {}\n')
    args = SimpleNamespace(output=str(prefix), workspace=tmp_path, backend='simulation',
                           geometry_config=None, calibration_config=None, timeout=2.)
    monkeypatch.setattr(map_bundle, '_git', lambda _: {'workspace': str(tmp_path)})
    monkeypatch.setattr(map_bundle, '_parameters', lambda *args: {'mode': 'mapping'})
    monkeypatch.setattr(map_bundle.rclpy, 'create_node', lambda _: SimpleNamespace(destroy_node=lambda: None))
    fail = True

    def raster(cmd, **kwargs):
        stage = Path(cmd[cmd.index('-f')+1])
        assert stage != prefix and stage.name == prefix.name
        Path(str(stage)+'.pgm').write_bytes(b'P5\n1 1\n255\n\0')
        # A map saver emitting an absolute image path must not refer to the
        # removed private staging directory after publication.
        Path(str(stage)+'.yaml').write_text('image: '+str(stage)+'.pgm\n')
        assert not Path(str(prefix)+'.yaml').exists()
        with pytest.raises(RuntimeError, match='in progress'):
            map_bundle._save(args)
        return SimpleNamespace(returncode=0)

    def graph(node, service, endpoint, request, timeout):
        if fail:
            return SimpleNamespace(result=1)
        for suffix in ('.posegraph', '.data'):
            Path(request.filename+suffix).write_bytes(b'graph')
        return SimpleNamespace(result=service.Response.RESULT_SUCCESS)

    monkeypatch.setattr(map_bundle.subprocess, 'run', raster)
    monkeypatch.setattr(map_bundle, '_call', graph)
    with pytest.raises(RuntimeError, match='Pose graph save failed'):
        map_bundle._save(args)
    assert list(prefix.parent.iterdir()) == []
    fail = False
    map_bundle._save(args)
    assert Path(str(prefix)+'.yaml').read_text() == 'image: warehouse.pgm\n'
    metadata = json.loads(Path(str(prefix)+'.metadata.json').read_text())
    previous = {ext: Path(str(prefix)+ext).read_bytes() for ext in BUNDLE_SUFFIXES}
    for suffix in ('yaml', 'pgm', 'posegraph', 'data'):
        target = Path(metadata['files'][suffix]['path'])
        assert target == Path(str(prefix)+'.'+suffix) and target.is_file()
    with pytest.raises(ValueError, match='Refusing to overwrite'):
        map_bundle._save(args)
    for ext, data in previous.items():
        assert Path(str(prefix)+ext).read_bytes() == data


def test_publication_rolls_back_only_own_links_on_collision(tmp_path):
    prefix = tmp_path / 'map'
    suffixes = ('.pgm', '.metadata.json', '.yaml')
    with pytest.raises(FileExistsError):
        with staged_map(prefix, suffixes) as stage:
            for suffix in suffixes:
                Path(str(stage)+suffix).write_text('{"files": {}}' if suffix == '.metadata.json' else 'map')
            Path(str(prefix)+'.metadata.json').write_text('another writer')
    assert sorted(p.name for p in tmp_path.iterdir()) == ['map.metadata.json']
    assert Path(str(prefix)+'.metadata.json').read_text() == 'another writer'


def test_interrupted_publication_recovers_owned_links_only(tmp_path):
    prefix = tmp_path / 'map'
    old = tmp_path / '.map.saving' / 'attempt-interrupted' / 'map'
    old.parent.mkdir(parents=True)
    Path(str(old)+'.pgm').write_text('previous attempt')
    os.link(str(old)+'.pgm', str(prefix)+'.pgm')
    with staged_map(prefix, ('.pgm', '.metadata.json', '.yaml')) as stage:
        assert not old.parent.exists() and not Path(str(prefix)+'.pgm').exists()
        for suffix in ('.pgm', '.metadata.json', '.yaml'):
            Path(str(stage)+suffix).write_text('{"files": {}}' if suffix == '.metadata.json' else 'new')
    assert Path(str(prefix)+'.pgm').read_text() == 'new'



def test_graph_reader_cannot_open_incomplete_publication(tmp_path):
    prefix = tmp_path / 'map'
    with pytest.raises(ValueError, match='has not completed'):
        with staged_map(prefix, BUNDLE_SUFFIXES) as stage:
            # These two links precede the metadata and public YAML at commit.
            for ext in ('.posegraph', '.data'):
                Path(str(stage)+ext).write_bytes(b'graph')
                os.link(str(stage)+ext, str(prefix)+ext)
            map_bundle._load(SimpleNamespace(input=str(prefix)))
    assert list(tmp_path.iterdir()) == []


@pytest.mark.parametrize('error', [ExternalShutdownException(), RuntimeError('Service response timed out')])
def test_shutdown_during_pending_service_rolls_back_and_releases_reservation(tmp_path, monkeypatch, capsys, error):
    import rclpy
    prefix = tmp_path / 'maps' / 'warehouse'
    (tmp_path / 'dependencies.repos').write_text('repositories: {}\n')
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'OFF')
    monkeypatch.setattr(map_bundle, '_git', lambda _: {'workspace': str(tmp_path)})
    monkeypatch.setattr(map_bundle, '_parameters', lambda *args: {'mode': 'mapping'})

    def raster(cmd, **kwargs):
        stage = Path(cmd[cmd.index('-f')+1])
        Path(str(stage)+'.pgm').write_bytes(b'P5\n1 1\n255\n\0')
        Path(str(stage)+'.yaml').write_text('image: warehouse.pgm\n')
        return SimpleNamespace(returncode=0)

    def pending(node, *args):
        # A signal shuts the context down while the graph service is pending.
        node.context.try_shutdown()
        raise error

    monkeypatch.setattr(map_bundle.subprocess, 'run', raster)
    monkeypatch.setattr(map_bundle, '_call', pending)
    argv = ['--output', str(prefix), '--workspace', str(tmp_path), '--timeout', '2']
    assert map_bundle.save_main(argv) == 1
    assert not rclpy.ok()
    # No public member, staging attempt or reservation lock remains.
    assert list(prefix.parent.iterdir()) == []
    message = capsys.readouterr().err
    assert ('interrupted' if isinstance(error, ExternalShutdownException) else 'timed out') in message
