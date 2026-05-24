# Pool Simulator

Physics-based 2D pool simulation and shot-planning engine in Python. The
project is written as a portfolio-quality mechanics implementation: physics is
derived in the code and documentation instead of delegated to a ready-made pool
or billiards simulator package.

## Status

**Phase 1 complete:** single-ball motion with manually derived rolling
resistance, configurable table parameters, closed-form timestep updates, and
pytest validation.

Upcoming: ball-ball collisions, rails, spin, visualization, inverse shot
optimization.

## Requirements

- Python 3.10+
- NumPy

## Install

```bash
cd "Pool Simulator"
python -m venv .venv
.venv\Scripts\activate
pip install -e ".[dev]"
```

## Run tests

```bash
pytest
```

## Run Phase 1 example

```bash
python examples/phase1_single_ball.py
```

## Project layout

```text
src/sim_core/
  physics/       # Derived mechanics and simulation loop
  rendering/     # Phase 6
  optimization/  # Phase 7
  utils/         # Vectors and constants
tests/           # pytest suite
docs/            # Physics derivations and assumptions
examples/        # Runnable demos
```

See [docs/physics.md](docs/physics.md) for units, derivations, and assumptions.

## License

TBD
