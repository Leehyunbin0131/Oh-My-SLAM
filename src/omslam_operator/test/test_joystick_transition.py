"""Exercise the QML pointer grab while the owner/mode service response is delayed."""
import os
import subprocess
import sys

def test_held_joystick_survives_editability_change_and_releases():
    environment = dict(os.environ, QT_QPA_PLATFORM='offscreen', QT_QUICK_BACKEND='software')
    subprocess.run([sys.executable, __file__], env=environment, check=True, timeout=20)

if __name__ == '__main__':
    from PySide6.QtCore import QObject, QPointF, Qt, QUrl
    from PySide6.QtGui import QGuiApplication
    from PySide6.QtQml import QQmlApplicationEngine, qmlRegisterType
    from PySide6.QtQuick import QQuickItem
    from PySide6.QtQuickControls2 import QQuickStyle
    from PySide6.QtTest import QTest
    from omslam_operator.app import qml_path
    from omslam_operator.visuals import CameraProvider, RobotMap
    import test_live_model as fixtures
    import time

    app = QGuiApplication([])
    QQuickStyle.setStyle('Basic')
    gen = fixtures.model.__wrapped__()
    model = next(gen)
    model.refresh()
    engine = QQmlApplicationEngine()
    engine.rootContext().setContextProperty('operatorModel', model)
    engine.rootContext().setContextProperty('defaultMapDirectory', '/tmp')
    engine.addImageProvider('camera', CameraProvider(model.s))
    qmlRegisterType(RobotMap, 'OhMyRobot', 1, 0, 'NativeMap')
    engine.load(QUrl.fromLocalFile(str(qml_path())))
    window = engine.rootObjects()[0]
    window.requestActivate()
    app.processEvents()
    model.setActive(True)
    pad = window.findChild(QObject, 'directPad')
    joystick = window.findChild(QQuickItem, 'joystickArea')
    position = joystick.mapToScene(QPointF(joystick.width()/2, joystick.height()/2-30)).toPoint()
    QTest.mousePress(window, Qt.LeftButton, Qt.NoModifier, position)
    fixtures.fresh(model)
    model.refresh()
    app.processEvents()
    assert joystick.property('pressed') and pad.property('held') and model._held
    assert model._pending['kind'] == 'manual'
    fixtures.reply(model)
    fixtures.reply(model, 'MANUAL')
    model._neutral_at = time.monotonic()-1.
    model.tick()
    assert model.s.latest_manual.deadman and model.s.latest_manual.command.twist.linear.x > 0.
    QTest.mouseRelease(window, Qt.LeftButton, Qt.NoModifier, position)
    assert not model._held and not model.s.latest_manual.deadman
    # Avoid Qt destroying the Python image provider after the test model.
    os._exit(0)
