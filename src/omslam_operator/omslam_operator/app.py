"""Native Qt console connected to ROS through a bounded background worker."""
import argparse
import json
import os
from pathlib import Path
import signal
import sys
import time

from PySide6.QtCore import QTimer, QUrl
from PySide6.QtGui import QGuiApplication
from PySide6.QtQml import QQmlApplicationEngine, qmlRegisterType
from PySide6.QtQuickControls2 import QQuickStyle


def qml_path():
    package = Path(__file__).resolve().parents[1]
    candidates = [package / "qml" / "design" / "Main.qml"]
    for parent in Path(__file__).absolute().parents:
        candidates.append(parent / "share" / "omslam_operator" / "qml" / "design" / "Main.qml")
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    raise FileNotFoundError("설치된 UI 파일이 없습니다. omslam_operator를 빌드하세요.")


def close_session(model, shared, worker):
    """Keep ROS transport alive briefly for an owned stop, with UI liveness ended."""
    owned = bool(model.token)
    model.timer.stop()
    model.display_timer.stop()
    model.shutdown()
    if owned:
        deadline = time.monotonic() + 2.0
        while worker.is_alive() and time.monotonic() < deadline:
            with shared.lock:
                state = shared.data.get('operator', {})
                received = shared.data.get('topics', {}).get('/operator/state', {}).get('received', 0.)
                done = (time.monotonic() - received < .3 and state.get('stopped')
                        and not state.get('base_active', True) and not state.get('owner_id'))
            if done:
                break
            time.sleep(.02)
    shared.stop.set()
    worker.join(timeout=3.)


def main(argv=None):
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.utilities import remove_ros_args
    from .bootstrap import workspace_root
    from .live_model import LiveConsoleModel
    from .visuals import CameraProvider, RobotMap
    from .worker import Shared, RosWorker

    raw_args = list(sys.argv[1:] if argv is None else argv)
    parser = argparse.ArgumentParser(description="Oh My Robot Console — ROS 연결")
    parser.add_argument('--page', type=int, default=0, choices=range(2))
    parser.add_argument('--width', type=int, default=1440)
    parser.add_argument('--height', type=int, default=900)
    clock = parser.add_mutually_exclusive_group()
    clock.add_argument('--use-sim-time', dest='use_sim_time', action='store_true')
    clock.add_argument('--real-time', dest='use_sim_time', action='store_false')
    parser.set_defaults(use_sim_time=True)
    parser.add_argument('--read-only', action='store_true', help='관측만 허용')
    parser.add_argument('--namespace', default='')
    parser.add_argument('--screenshot')
    parser.add_argument('--screenshot-delay', type=float, default=1.0)
    parser.add_argument('--snapshot')
    parser.add_argument('--capture-when-control-fresh', action='store_true',
                        help='최신 제어 상태를 받은 뒤 화면을 기록')
    parser.add_argument('--path-debug-directory', default='',
                        help='세션이 시작한 경로 기록 디렉터리')
    parser.add_argument('--quit-after', type=float, default=0.0)
    args = parser.parse_args(remove_ros_args(args=['omslam_operator', *raw_args])[1:])

    QQuickStyle.setStyle('Basic')
    app = QGuiApplication([sys.argv[0]])
    app.setApplicationName('Oh My Robot Console')
    app.setOrganizationName('Oh My MobileRobot')
    rclpy.init(args=['omslam_operator', *raw_args], signal_handler_options=SignalHandlerOptions.NO)
    shared = Shared(path_debug_directory=args.path_debug_directory)
    worker = RosWorker(shared, args.use_sim_time, args.namespace)
    worker.start()
    model = LiveConsoleModel(shared, initial_page=args.page, read_only=args.read_only)
    qmlRegisterType(RobotMap, 'OhMyRobot', 1, 0, 'NativeMap')
    engine = QQmlApplicationEngine()
    engine.rootContext().setContextProperty('operatorModel', model)
    root = Path(os.environ.get('OMSLAM_CONSOLE_ROOT') or workspace_root() or Path.cwd())
    engine.rootContext().setContextProperty('defaultMapDirectory', str(root / 'maps' / 'operator'))
    engine.addImageProvider('camera', CameraProvider(shared))
    engine.setInitialProperties({'page': args.page, 'width': max(1280, args.width),
                                 'height': max(800, args.height)})
    capture_failed = False
    try:
        engine.load(QUrl.fromLocalFile(str(qml_path())))
        if not engine.rootObjects():
            return 1
        window = engine.rootObjects()[0]
        window.activeChanged.connect(lambda: model.setActive(window.isActive()))
        model.setActive(window.isActive())

        def capture():
            nonlocal capture_failed
            if args.capture_when_control_fresh and not model.ui.get('controlFresh'):
                QTimer.singleShot(50, capture)
                return
            try:
                if args.screenshot:
                    target = Path(args.screenshot).expanduser()
                    target.parent.mkdir(parents=True, exist_ok=True)
                    if not window.grabWindow().save(str(target)):
                        raise RuntimeError('스크린샷을 저장하지 못했습니다.')
                if args.snapshot:
                    target = Path(args.snapshot).expanduser()
                    target.parent.mkdir(parents=True, exist_ok=True)
                    target.write_text(json.dumps({'preview': False, 'ui': model.ui,
                        'goals': model.goals, 'page': window.property('page'),
                        'width': window.width(), 'height': window.height(),
                        'use_sim_time': args.use_sim_time, 'read_only': args.read_only},
                        ensure_ascii=False, indent=2) + '\n')
            except Exception as exc:
                print('화면 기록 실패: ' + str(exc), file=sys.stderr)
                capture_failed = True

        if args.screenshot or args.snapshot:
            QTimer.singleShot(max(0, int(args.screenshot_delay * 1000)), capture)
        if args.quit_after:
            QTimer.singleShot(int(args.quit_after * 1000), app.quit)
        signal.signal(signal.SIGINT, lambda *_: app.quit())
        signal.signal(signal.SIGTERM, lambda *_: app.quit())
        signal_timer = QTimer()
        signal_timer.timeout.connect(lambda: None)
        signal_timer.start(250)
        result = app.exec()
        return 1 if capture_failed else result
    finally:
        close_session(model, shared, worker)
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
