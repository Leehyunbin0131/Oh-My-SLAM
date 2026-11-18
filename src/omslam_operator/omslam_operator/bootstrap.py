"""Launch the installed console with the workspace's pinned Qt interpreter."""
import importlib.util
import os
from pathlib import Path
import sys


def workspace_root():
    location = Path(__file__).absolute()
    for parent in location.parents:
        if parent.name == 'install':
            return parent.parent
    # Also support the workspace's symlink-install layout.
    for parent in location.resolve().parents:
        if (parent / 'src' / 'omslam_operator').is_dir():
            return parent
    return None


def main(argv=None):
    if importlib.util.find_spec('PySide6') is not None:
        from .app import main as app_main
        return app_main(argv)
    root = workspace_root()
    interpreter = root / '.venv-operator' / 'bin' / 'python' if root else None
    if interpreter and interpreter.is_file() and os.access(interpreter, os.X_OK):
        if Path(sys.prefix).absolute() != interpreter.parent.parent.absolute():
            os.execv(str(interpreter), [str(interpreter), '-m', 'omslam_operator.app',
                                      *(sys.argv[1:] if argv is None else argv)])
    directory = str(root) if root else 'Oh_My_SLAM workspace'
    print('Qt 실행 환경이 없습니다. 다음 작업 경로에서 GUI 의존성을 설치하세요: ' + directory,
          file=sys.stderr)
    print('python3 -m venv --system-site-packages .venv-operator', file=sys.stderr)
    print('.venv-operator/bin/python -m pip install -r requirements-operator.txt', file=sys.stderr)
    return 1


if __name__ == '__main__':
    sys.exit(main())
