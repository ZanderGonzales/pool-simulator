"""Phase 1 demo: single ball with derived rolling resistance."""

from sim_core.physics.ball import Ball
from sim_core.physics.integrator import stopping_distance, stopping_time
from sim_core.physics.simulation import Simulation, SimulationConfig
from sim_core.physics.table import TableConfig
from sim_core.utils.vectors import vec2


def main() -> None:
    table = TableConfig(rolling_resistance_coefficient=0.02)
    ball = Ball(id=0, position=vec2(0.5, 0.25), velocity=vec2(2.0, 0.0))
    initial_speed = ball.speed
    sim = Simulation(
        balls=[ball],
        table=table,
        config=SimulationConfig(dt=0.01),
    )

    print("Phase 1 - single ball motion")
    print(f"Table: {table.width:.2f} m x {table.height:.2f} m")
    print(
        "Rolling resistance "
        f"mu_r={table.rolling_resistance_coefficient}, dt={sim.config.dt} s"
    )
    print(
        "Closed form from initial speed: "
        f"t_stop={stopping_time(initial_speed, table):.3f} s, "
        f"d_stop={stopping_distance(initial_speed, table):.3f} m\n"
    )

    for step in range(501):
        if step % 50 == 0:
            print(
                f"t={sim.time:6.3f}s  "
                f"pos=({ball.position[0]:.4f}, {ball.position[1]:.4f})  "
                f"vel=({ball.velocity[0]:.4f}, {ball.velocity[1]:.4f})  "
                f"speed={ball.speed:.4f} m/s"
            )
        if step < 500:
            sim.step()

    print(f"\nStopped: {sim.all_stopped()}")


if __name__ == "__main__":
    main()
