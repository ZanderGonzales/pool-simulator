"""Simulation diagnostics for tests and regression checks."""

from __future__ import annotations

from sim_core.physics.ball import Ball
from sim_core.utils.vectors import norm


def total_kinetic_energy(balls: list[Ball]) -> float:
    """Return total translational kinetic energy of active balls."""
    return sum(0.5 * ball.mass * ball.speed**2 for ball in balls if ball.active)


def max_ball_overlap(balls: list[Ball]) -> float:
    """Return the maximum pairwise penetration depth across active balls."""
    max_penetration = 0.0
    for i in range(len(balls)):
        ball_a = balls[i]
        if not ball_a.active:
            continue
        for j in range(i + 1, len(balls)):
            ball_b = balls[j]
            if not ball_b.active:
                continue
            min_distance = ball_a.radius + ball_b.radius
            distance = norm(ball_b.position - ball_a.position)
            penetration = max(0.0, min_distance - distance)
            max_penetration = max(max_penetration, penetration)
    return max_penetration


def count_moving_balls(balls: list[Ball], velocity_stop_threshold: float) -> int:
    """Return the number of active balls still moving above the stop threshold."""
    return sum(
        1 for ball in balls if ball.is_moving(velocity_stop_threshold)
    )
