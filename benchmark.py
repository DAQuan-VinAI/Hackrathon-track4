"""Entry point for the complete vinsort implementation (not the root prototype)."""
from pathlib import Path
import runpy
import sys

if __name__ == "__main__":
    source = Path(__file__).resolve().parent / "vinsort"
    sys.path.insert(0, str(source))
    runpy.run_path(str(source / "benchmark.py"), run_name="__main__")
