"""Phase 7 demo: side spin, 3D angular velocity, and swerve."""

from sim_core.physics.ball import Ball
from sim_core.physics.diagnostics import total_energy
from sim_core.physics.rack import create_break_setup
from sim_core.physics.shot import CueStrike
from sim_core.physics.simulation import Simulation, SimulationConfig
from sim_core.physics.table import TableConfig
from sim_core.utils.vectors import vec2, vec3


def _run_side_spin_roll() -> None:
    table = TableConfig(
        rolling_resistance_coefficient=0.01,
        sliding_friction_coefficient=0.15,
        swerve_coefficient=0.08,
    )
    ball = Ball(
        id=0,
        position=vec2(0.3, table.height * 0.5),
        velocity=vec2(1.5, 0.0),
        angular_velocity=vec3(0.0, 15.0, 0.0),
    )
    sim = Simulation(balls=[ball], table=table, config=SimulationConfig(dt=0.01))

    print("\nSide spin roll (omega_y, swerve on)")
    for step in range(201):
        if step % 40 == 0:
            av = ball.angular_velocity
            print(
                f"  t={sim.time:5.2f}s  pos=({ball.position[0]:.3f}, {ball.position[1]:.3f})  "
                f"v=({ball.velocity[0]:.2f}, {ball.velocity[1]:.2f})  "
                f"omega=({av[0]:.1f}, {av[1]:.1f}, {av[2]:.1f})"
            )
        if step < 200:
            sim.step()


def _run_break_with_side_english() -> None:
    table = TableConfig()
    strike = CueStrike(
        speed=3.0,
        aim_direction=vec2(-1.0, 0.0),
        hit_offset_perpendicular=0.3,
    )
    setup = create_break_setup(cue_strike=strike)
    sim = Simulation(
        balls=list(setup.balls),
        table=table,
        config=SimulationConfig(dt=0.005, collision_iterations=6),
    )
    cue = sim.balls[setup.cue_ball_index]

    print("\nBreak with side english (perpendicular cue offset)")
    for step in range(301):
        if step % 60 == 0:
            av = cue.angular_velocity
            print(
                f"  t={sim.time:5.2f}s  cue_v=({cue.velocity[0]:.2f}, {cue.velocity[1]:.2f})  "
                f"omega=({av[0]:.1f}, {av[1]:.1f}, {av[2]:.1f})  E={total_energy(sim.balls):.2f} J"
            )
        if step < 300:
            sim.step()


def main() -> None:
    print("Phase 7 - 3D angular velocity and side spin")
    _run_side_spin_roll()
    _run_break_with_side_english()


if __name__ == "__main__":
    main()
