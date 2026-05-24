# Pool Simulator

Physics-based 2D pool simulation and shot-planning engine in Python. The
project is written as a portfolio-quality mechanics implementation: physics is
derived in the code and documentation instead of delegated to a ready-made pool
or billiards simulator package.

## Status

**Phase 1 complete:** single-ball motion with manually derived rolling
resistance, configurable table parameters, closed-form timestep updates, and
pytest validation.

**Phase 2 complete:** ball-ball collision detection, impulse-based resolution
with configurable restitution, positional separation, and conservation tests.

**Phase 3 complete:** cushion segments, closest-point rail detection, velocity
reflection with restitution, optional tangential damping, and anti-tunneling
positional correction.

**Phase 4 complete:** 15-ball triangle rack layout, break-shot setup, multi-ball
integration tests, and simulation diagnostics.

Upcoming: spin, visualization, inverse shot optimization.

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

## Run examples

```bash
python examples/phase1_single_ball.py
python examples/phase2_ball_collision.py
python examples/phase3_rail_bounce.py
python examples/phase4_break_shot.py
```

## Project layout

```text
src/sim_core/
  physics/       # Derived mechanics and simulation loop
    collision/   # detector/resolver (balls) and rail_detector/rail_resolver
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
