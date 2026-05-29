"""Closed-form rolling-resistance integration for one billiard ball."""

from __future__ import annotations

from sim_core.physics.ball import Ball
from sim_core.physics.table import TableConfig
from sim_core.utils.constants import GRAVITY_MPS2


def rolling_deceleration(table: TableConfig) -> float:
    """
    Return the acceleration magnitude caused by rolling resistance.

    The model is F_r = -mu_r N v_hat. On a level table N = mg, so dividing
    by mass gives a = -mu_r g v_hat. Mass cancels because heavier balls also
    press proportionally harder into the cloth.
    """
    return table.rolling_resistance_coefficient * GRAVITY_MPS2


def stopping_time(speed: float, table: TableConfig) -> float:
    """Return t_stop = v0 / a for constant deceleration."""
    deceleration = rolling_deceleration(table)
    if deceleration == 0.0:
        return float("inf")
    return max(0.0, speed) / deceleration


def stopping_distance(speed: float, table: TableConfig) -> float:
    """Return d_stop = v0^2 / (2a) for constant deceleration."""
    deceleration = rolling_deceleration(table)
    if deceleration == 0.0:
        return float("inf")
    speed = max(0.0, speed)
    return speed * speed / (2.0 * deceleration)


def apply_rolling_resistance(ball: Ball, table: TableConfig, dt: float) -> None:
    """Advance position and velocity with exact constant-deceleration equations."""
    if not ball.active:
        return
    if dt <= 0.0:
        raise ValueError("dt must be positive")

    speed = ball.speed
    if speed < table.velocity_stop_threshold:
        ball.stop()
        return

    direction = ball.velocity / speed
    deceleration = rolling_deceleration(table)
    if deceleration == 0.0:
        integrate_position(ball, dt)
        return

    t_stop = speed / deceleration
    if t_stop <= dt:
        distance = speed * t_stop - 0.5 * deceleration * t_stop * t_stop
        ball.position = ball.position + direction * distance
        ball.stop()
        return

    distance = speed * dt - 0.5 * deceleration * dt * dt
    ball.position = ball.position + direction * distance
    ball.velocity = direction * (speed - deceleration * dt)


def apply_stopping(ball: Ball, table: TableConfig) -> None:
    """Clamp velocity to zero when below the stop threshold."""
    if not ball.active:
        return
    if ball.speed < table.velocity_stop_threshold:
        ball.stop()


def integrate_position(ball: Ball, dt: float) -> None:
    """Update position using current velocity."""
    if not ball.active:
        return
    ball.position = ball.position + ball.velocity * dt


def step_ball(ball: Ball, table: TableConfig, dt: float) -> None:
    """Single-timestep update with rolling/sliding cloth regimes."""
    from sim_core.physics.spin_integrator import step_ball_spin

    if ball.omega != 0.0:
        step_ball_spin(ball, table, dt)
        return
    apply_rolling_resistance(ball, table, dt)
    apply_stopping(ball, table)
