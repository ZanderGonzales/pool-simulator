"""Phase 6 demo: pockets, rail friction, cue strike offsets, and skidding."""

from sim_core.physics.ball import Ball
from sim_core.physics.diagnostics import count_pocketed_balls, count_moving_balls
from sim_core.physics.rack import create_break_setup
from sim_core.physics.shot import CueStrike
from sim_core.physics.simulation import Simulation, SimulationConfig
from sim_core.physics.table import TableConfig
from sim_core.utils.vectors import vec2


def _run_pocket_demo() -> None:
    table = TableConfig()
    corner = table.pockets[0].center
    ball = Ball(
        id=0,
        position=corner + vec2(0.04, 0.0),
        velocity=vec2(-0.8, 0.0),
    )
    sim = Simulation(balls=[ball], table=table, config=SimulationConfig(dt=0.01))

    print("\nPocket capture (corner pocket)")
    for step in range(201):
        if step % 40 == 0:
            print(
                f"  t={sim.time:5.2f}s  active={ball.active}  "
                f"pos=({ball.position[0]:.3f}, {ball.position[1]:.3f})"
            )
        if ball.active and step < 200:
            sim.step()
    print(f"  pocketed count={count_pocketed_balls(sim.balls)}")


def _run_skid_demo() -> None:
    table = TableConfig(
        rolling_resistance_coefficient=0.0,
        sliding_friction_coefficient=0.25,
        spin_decay_rate=0.0,
    )
    ball = Ball(
        id=0,
        position=vec2(0.3, table.height * 0.5),
        velocity=vec2(2.0, 0.0),
        omega=0.0,
    )
    sim = Simulation(balls=[ball], table=table, config=SimulationConfig(dt=0.01))

    print("\nSkidding ball (no spin, sliding friction only)")
    for step in range(151):
        if step % 30 == 0:
            print(f"  t={sim.time:5.2f}s  speed={ball.speed:.3f} m/s  omega={ball.omega:.3f}")
        if step < 150:
            sim.step()


def _run_break_with_cue_strike() -> None:
    table = TableConfig()
    strike = CueStrike(
        speed=3.2,
        aim_direction=vec2(-1.0, 0.0),
        hit_offset_parallel=0.35,
    )
    setup = create_break_setup(cue_strike=strike)
    sim = Simulation(
        balls=list(setup.balls),
        table=table,
        config=SimulationConfig(dt=0.005, collision_iterations=6),
    )
    cue = sim.balls[setup.cue_ball_index]

    print("\nBreak with CueStrike follow offset")
    for step in range(401):
        if step % 80 == 0:
            moving = count_moving_balls(
                sim.balls,
                table.velocity_stop_threshold,
                table.omega_stop_threshold,
            )
            print(
                f"  t={sim.time:5.2f}s  cue_speed={cue.speed:.3f}  cue_omega={cue.omega:.2f}  "
                f"moving={moving}  pocketed={count_pocketed_balls(sim.balls)}"
            )
        if step < 400:
            sim.step()


def main() -> None:
    print("Phase 6 - pockets, rail friction defaults, cue strike, skidding path")
    _run_pocket_demo()
    _run_skid_demo()
    _run_break_with_cue_strike()


if __name__ == "__main__":
    main()
