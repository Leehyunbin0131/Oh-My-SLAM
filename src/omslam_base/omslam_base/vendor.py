"""Import only bundled, pinned host modules from the installed package."""
import importlib
import sys
from pathlib import Path
from ament_index_python.packages import get_package_share_directory


def load_control_module(name):
    if name not in {'wheel_drive', 'motor_link', 'motor_pid', 'drive_calibration', 'serial_port', 'hfi_a9'}:
        raise ValueError('Not a bundled control module')
    directory = Path(get_package_share_directory('omslam_control_vendor')) / 'control'
    expected = (directory / (name + '.py')).resolve()
    if name in sys.modules and Path(sys.modules[name].__file__).resolve() != expected:
        raise RuntimeError('A different control module is already imported: ' + name)
    # Existing upstream modules import each other by their original names.
    # Resolve those imports solely from this explicitly installed dependency.
    sys.path.insert(0, str(directory))
    try:
        return importlib.import_module(name)
    finally:
        sys.path.remove(str(directory))
