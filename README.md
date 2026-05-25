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

**Phase 4 complete:** 15-ball triangle rack layout, break-shot setup,
post-collision cleanup for dense racks, multi-ball integration tests, and
simulation diagnostics.

**Phase 5 complete:** scalar spin (`omega`), slip-based cloth friction with
rolling/sliding regimes, ball-ball tangential impulse with Coulomb cap, cue
english via `ShotParams`, rotational diagnostics, and spin tests.

Upcoming: visualization (Phase 6), inverse shot optimization (Phase 7).

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
python examples/phase5_spin_demo.py
```

## Test coverage

The pytest suite is organized by physics feature rather than by implementation
file. It validates:

- closed-form rolling-resistance motion and exact stopping behavior
- ball-ball contact detection, separation, conservation, and restitution
- rail closest-point detection, anti-tunneling correction, and reflection
- 15-ball rack geometry, break-shot setup, energy monotonicity, and overlap
  cleanup
- spin slip velocity, cloth friction decay, tangential collision impulses, and
  cue english on break setup
- simulation bookkeeping such as time advancement, fixed-step runs,
  run-until-stopped, and snapshots

The tests intentionally do not assert rendering or optimizer behavior because
those phases are placeholders. They also do not claim real-world break-shot
accuracy yet because pockets and a dynamic cue impulse model are still out of
scope.

## Project layout

```text
src/sim_core/
  physics/       # Derived mechanics and simulation loop
  collision/   # detector/resolver (balls) and rail_detector/rail_resolver
  spin_integrator.py
  shot.py
  rack.py      # 15-ball rack and break-shot setup helpers
  diagnostics.py
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
