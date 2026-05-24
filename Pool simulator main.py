"""Legacy entry point - prefer: python examples/phase1_single_ball.py."""

from pathlib import Path
import runpy


if __name__ == "__main__":
    script = Path(__file__).resolve().parent / "examples" / "phase1_single_ball.py"
    runpy.run_path(str(script), run_name="__main__")
