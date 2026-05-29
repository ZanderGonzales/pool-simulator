from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from sim_core.physics.ball import Ball
from sim_core.physics.collision.detector import find_ball_ball_contacts
from sim_core.physics.collision.rail_detector import find_ball_rail_contacts
from sim_core.physics.collision.rail_resolver import resolve_ball_rail_contacts
from sim_core.physics.collision.resolver import resolve_ball_ball_contacts, separate_balls
from sim_core.physics.integrator import step_ball
from sim_core.physics.table import TableConfig
from sim_core.utils.vectors import norm


@dataclass
class SimulationConfig:
    """Timestep and safety limits for the simulation loop."""

    dt: float = 0.01
    max_steps: int = 100_000
    collision_iterations: int = 4

    def __post_init__(self) -> None:
        if self.dt <= 0:
            raise ValueError("dt must be positive")
        if self.max_steps <= 0:
            raise ValueError("max_steps must be positive")
        if self.collision_iterations <= 0:
            raise ValueError("collision_iterations must be positive")


@dataclass
class Simulation:
    """2D billiards simulation with rolling resistance and collisions."""

    balls: list[Ball]
    table: TableConfig
    config: SimulationConfig = field(default_factory=SimulationConfig)
    time: float = 0.0

    def step(self) -> None:
        """Advance one timestep, then resolve ball-ball and rail contacts."""
        dt = self.config.dt
        for ball in self.balls:
            step_ball(ball, self.table, dt)
        self._resolve_ball_ball_collisions()
        self._resolve_rail_collisions()
        self._resolve_ball_ball_collisions()
        self._resolve_pockets()
        self.time += dt

    def _resolve_ball_ball_collisions(self) -> None:
        """Iteratively detect and resolve overlaps within the same timestep."""
        for _ in range(self.config.collision_iterations):
            contacts = find_ball_ball_contacts(self.balls)
            if not contacts:
                break
            resolve_ball_ball_contacts(self.balls, contacts, self.table)

        for _ in range(self.config.collision_iterations):
            contacts = find_ball_ball_contacts(self.balls)
            overlapping = [contact for contact in contacts if contact.penetration > 0.0]
            if not overlapping:
                break
            for contact in sorted(overlapping, key=lambda item: (item.index_a, item.index_b)):
                ball_a = self.balls[contact.index_a]
                ball_b = self.balls[contact.index_b]
                separate_balls(ball_a, ball_b, contact.normal, contact.penetration)

    def _resolve_rail_collisions(self) -> None:
        """Iteratively resolve ball-cushion penetrations within the same timestep."""
        for _ in range(self.config.collision_iterations):
            contacts = find_ball_rail_contacts(self.balls, self.table)
            if not contacts:
                break
            resolve_ball_rail_contacts(self.balls, contacts, self.table)

    def _resolve_pockets(self) -> None:
        """Deactivate balls captured by any pocket region."""
        for ball in self.balls:
            if not ball.active:
                continue
            if (
                ball.position[0] < 0.0
                or ball.position[0] > self.table.width
                or ball.position[1] < 0.0
                or ball.position[1] > self.table.height
            ):
                continue
            for pocket in self.table.pockets:
                if norm(ball.position - pocket.center) <= pocket.radius:
                    ball.active = False
                    ball.stop()
                    break

    def all_stopped(self) -> bool:
        return all(
            not ball.is_moving(
                self.table.velocity_stop_threshold,
                self.table.omega_stop_threshold,
            )
            for ball in self.balls
            if ball.active
        )

    def run(
        self,
        steps: int | None = None,
        *,
        until_stopped: bool = False,
    ) -> int:
        """
        Run the simulation for a fixed number of steps or until all balls stop.

        Returns the number of steps executed.
        """
        if steps is not None and steps < 0:
            raise ValueError("steps must be non-negative")
        if until_stopped and steps is not None:
            raise ValueError("specify either steps or until_stopped, not both")

        executed = 0
        limit = steps if steps is not None else self.config.max_steps

        while executed < limit:
            if until_stopped and self.all_stopped():
                break
            self.step()
            executed += 1
            if until_stopped and self.all_stopped():
                break

        return executed

    def snapshot(self) -> dict[str, Any]:
        """Serializable state for rendering or diagnostics."""
        return {
            "time": self.time,
            "balls": [
                {
                    "id": ball.id,
                    "position": ball.position.copy(),
                    "velocity": ball.velocity.copy(),
                    "omega": ball.omega,
                    "active": ball.active,
                    "radius": ball.radius,
                }
                for ball in self.balls
            ],
            "table": {
                "width": self.table.width,
                "height": self.table.height,
            },
        }
