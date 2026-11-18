#!/usr/bin/env python3
"""Compatibility entry point for the installed Oh My Robot Console verifier."""
import os
from pathlib import Path
import sys


if __name__ == '__main__':
    verifier = Path(__file__).with_name('verify_operator_live.py')
    os.execv(sys.executable, [sys.executable, str(verifier), *sys.argv[1:]])
