"""Phase 5 demo: top spin, draw, and break shot with cue english."""

from sim_core.physics.ball import Ball
from sim_core.physics.diagnostics import rotational_kinetic_energy, total_energy
from sim_core.physics.rack import create_break_setup
from sim_core.physics.shot import ShotParams
from sim_core.physics.simulation import Simulation, SimulationConfig
from sim_core.physics.table import TableConfig
from sim_core.utils.constants import BALL_RADIUS_M
from sim_core.utils.vectors import vec2


def _run_single_ball(label: str, omega: float, *, steps: int = 300) -> None:
    table = TableConfig(
        rolling_resistance_coefficient=0.02,
        sliding_friction_coefficient=0.25,
        spin_decay_rate=3.0,
    )
    speed = 1.0
    ball = Ball(
        id=0,
        position=vec2(0.5, table.height * 0.5),
        velocity=vec2(speed, 0.0),
        omega=omega,
    )
    sim = Simulation(balls=[ball], table=table, config=SimulationConfig(dt=0.01))

    print(f"\n{label} (omega={omega:.1f} rad/s)")
    for step in range(steps + 1):
        if step % 60 == 0:
            print(
                f"  t={sim.time:5.2f}s  pos=({ball.position[0]:.3f}, {ball.position[1]:.3f})  "
                f"speed={ball.speed:.3f} m/s  omega={ball.omega:.2f} rad/s  "
                f"E_rot={rotational_kinetic_energy(sim.balls):.5f} J"
            )
        if step < steps:
            sim.step()


def _run_break(omega_cue: float) -> None:
    table = TableConfig()
    setup = create_break_setup(shot=ShotParams(speed=3.0, omega=omega_cue))
    sim = Simulation(
        balls=list(setup.balls),
        table=table,
        config=SimulationConfig(dt=0.005, collision_iterations=6),
    )
    cue = sim.balls[setup.cue_ball_index]

    print(f"\nBreak with cue omega={omega_cue:.1f} rad/s")
    for step in range(401):
        if step % 80 == 0:
            print(
                f"  t={sim.time:5.2f}s  cue_speed={cue.speed:.3f} m/s  "
                f"cue_omega={cue.omega:.2f}  total_E={total_energy(sim.balls):.3f} J"
            )
        if step < 400:
            sim.step()


def main() -> None:
    r = BALL_RADIUS_M
    print("Phase 5 - spin and rolling/sliding dynamics")
    print(f"Ball radius {r:.5f} m; top spin omega=+{1.0/r:.0f} rad/s, draw omega=-{1.0/r:.0f} rad/s")

    _run_single_ball("No spin", 0.0)
    _run_single_ball("Top spin (follow)", 1.0 / r)
    _run_single_ball("Draw (check)", -1.0 / r)

    _run_break(0.0)
    _run_break(8.0)


if __name__ == "__main__":
    main()
