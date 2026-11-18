"""Map bindings must not silently discard saved keepout restrictions."""
import hashlib
import json
from pathlib import Path

import pytest
import yaml

from omslam_navigation.launch_support import keepout_file_for_map, validate_map_bundle


def map_bundle(directory, name='map'):
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / (name + '.yaml')
    image = directory / (name + '.pgm')
    image.write_bytes(b'P5\n2 2\n255\n\xff\xff\xff\xff')
    path.write_text(yaml.safe_dump({'image': image.name, 'resolution': .05,
        'origin': [0., 0., 0.], 'negate': 0, 'occupied_thresh': .65, 'free_thresh': .25}))
    metadata = {'schema_version': 1, 'backend': 'simulation',
        'frames': {'map_frame': 'map', 'odom_frame': 'odom', 'base_frame': 'base_footprint'},
        'files': {key: {'sha256': hashlib.sha256(p.read_bytes()).hexdigest()}
                  for key, p in [('yaml', path), ('pgm', image)]}}
    path.with_suffix('.metadata.json').write_text(json.dumps(metadata))
    return path


def sidecar(path):
    image = path.with_suffix('.pgm')
    data = {'schema_version': 1, 'frame_id': 'map',
        'polygons': [[[0., 0.], [1., 0.], [1., 1.], [0., 1.]]],
        'map_sha256': {key: hashlib.sha256(p.read_bytes()).hexdigest()
                      for key, p in [('yaml', path), ('pgm', image)]}}
    path.with_suffix('.keepout.json').write_text(json.dumps(data))
    return data


def test_old_map_without_sidecar_is_valid_and_keepouts_restore_when_present(tmp_path):
    path = map_bundle(tmp_path)
    assert validate_map_bundle(path) == path
    assert keepout_file_for_map(path) is None
    sidecar(path)
    assert validate_map_bundle(path) == path
    assert keepout_file_for_map(path) == path.with_suffix('.keepout.json')


@pytest.mark.parametrize('change', ['hash', 'geometry', 'invalid_json', 'missing_binding'])
def test_present_invalid_sidecar_prevents_loading_map(tmp_path, change):
    path = map_bundle(tmp_path)
    data = sidecar(path)
    target = path.with_suffix('.keepout.json')
    if change == 'hash':
        data['map_sha256']['pgm'] = '0' * 64
    elif change == 'geometry':
        data['polygons'][0][2] = [2., 2.]
    elif change == 'missing_binding':
        data.pop('map_sha256')
    target.write_text('{' if change == 'invalid_json' else json.dumps(data))
    with pytest.raises(ValueError):
        validate_map_bundle(path)


def test_sidecar_cannot_be_reused_after_map_changes_even_with_new_metadata(tmp_path):
    path = map_bundle(tmp_path)
    sidecar(path)
    image = path.with_suffix('.pgm')
    image.write_bytes(image.read_bytes()[:-1] + b'\x00')
    metadata = json.loads(path.with_suffix('.metadata.json').read_text())
    metadata['files']['pgm']['sha256'] = hashlib.sha256(image.read_bytes()).hexdigest()
    path.with_suffix('.metadata.json').write_text(json.dumps(metadata))
    with pytest.raises(ValueError, match='Keepout pgm hash'):
        validate_map_bundle(path)


def test_incomplete_save_cannot_be_loaded_as_legacy_map(tmp_path):
    path = map_bundle(tmp_path)
    marker = path.with_suffix('.keepout.pending')
    marker.write_text('incomplete')
    with pytest.raises(ValueError, match='incomplete'):
        validate_map_bundle(path)
    # Internal writer may verify raster provenance before finishing its sidecar.
    assert validate_map_bundle(path, allow_pending_keepouts=True) == path
    sidecar(path)
    with pytest.raises(ValueError, match='incomplete'):
        validate_map_bundle(path)
    marker.unlink()
    assert validate_map_bundle(path) == path
