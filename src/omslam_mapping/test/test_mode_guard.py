import multiprocessing

from omslam_mapping.mode_guard import ModeGuard


def _try_other_mode(directory, queue):
    try:
        guard = ModeGuard('localization', directory=directory, domain=123)
    except RuntimeError:
        queue.put('rejected')
    else:
        guard.close()
        queue.put('acquired')


def test_other_process_cannot_own_map_tf_until_mode_stops(tmp_path):
    context = multiprocessing.get_context('spawn')
    queue = context.Queue()
    guard = ModeGuard('mapping', directory=tmp_path, domain=123)
    process = context.Process(target=_try_other_mode, args=(str(tmp_path), queue))
    process.start()
    process.join(timeout=5)
    assert process.exitcode == 0
    assert queue.get(timeout=1) == 'rejected'
    guard.close()
    process = context.Process(target=_try_other_mode, args=(str(tmp_path), queue))
    process.start()
    process.join(timeout=5)
    assert process.exitcode == 0
    assert queue.get(timeout=1) == 'acquired'
    queue.close()
