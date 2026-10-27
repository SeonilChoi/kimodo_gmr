"""BDX entry point for SMPL-X to robot retargeting.

This keeps the implementation shared with smplx_to_robot.py while defaulting
to the BDX robot.
"""

from __future__ import annotations

import runpy
import sys
from pathlib import Path


def _normalize_args(argv: list[str]) -> list[str]:
    if any(arg == "--robot" or arg.startswith("--robot=") for arg in argv[1:]):
        return argv

    return [*argv, "--robot", "bdx"]


if __name__ == "__main__":
    target_script = Path(__file__).with_name("smplx_to_robot.py")
    normalized_argv = _normalize_args(sys.argv)

    original_argv = sys.argv
    try:
        sys.argv = [str(target_script), *normalized_argv[1:]]
        runpy.run_path(str(target_script), run_name="__main__")
    finally:
        sys.argv = original_argv
