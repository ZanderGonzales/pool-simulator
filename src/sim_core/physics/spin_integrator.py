"""Cloth friction and slip velocity for spin-aware ball motion."""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

from sim_core.physics.ball import Ball
from sim_core.physics.integrator import apply_rolling_resistance
from sim_core.physics.table import TableConfig
from sim_core.utils.constants import GRAVITY_MPS2
from sim_core.utils.vectors import norm, vec2

Vec2 = NDArray[np.float64]

_VELOCITY_EPS = 1e-12


def perpendicular_ccw(v: Vec2) -> Vec2:
    """Unit vector 90 degrees counter-clockwise from v (zero if v has zero length)."""
    length = norm(v)
    if length < _VELOCITY_EPS:
        return vec2(0.0, 0.0)
    return vec2(-v[1] / length, v[0] / length)


def slip_velocity(ball: Ball) -> Vec2:
    """
    Slip velocity at the cloth contact for a 2D rigid disk.

    Uses v_slip = v - omega_z * r * v_hat (motion-aligned slip). This lumped 2D
    model maps positive omega to top spin and negative omega to draw along the
    current velocity direction. When speed is near zero, a fixed reference axis
    avoids singularities. Side english and full contact-patch physics are not modeled.
    """
    v = ball.velocity
    speed = norm(v)
    r = ball.radius
    if speed < _VELOCITY_EPS:
        return v - ball.omega * r * vec2(1.0, 0.0)
    v_hat = v / speed
    return v - ball.omega * r * v_hat


def sphere_inertia(mass: float, radius: float) -> float:
    """Moment of inertia for a solid sphere about the vertical axis."""
    return 0.4 * mass * radius * radius


def apply_cloth_friction(ball: Ball, table: TableConfig, dt: float) -> None:
    """
    Update velocity and omega from cloth friction (sliding or rolling regime).

    Sliding: reduce slip speed by up to mu_s * g * dt along -slip_hat, coupled to spin.
    Rolling: when slip is small or spin exceeds rim speed, use rolling resistance and
    optional spin decay d_omega/dt = -k_omega * omega.
    """
    if not ball.active or dt <= 0.0:
        return

    slip = slip_velocity(ball)
    slip_speed = norm(slip)
    speed = ball.speed
    rim_speed = abs(ball.omega) * ball.radius

    if (
        speed < table.velocity_stop_threshold
        and abs(ball.omega) < table.omega_stop_threshold
    ):
        ball.stop()
        return

    if slip_speed < table.sliding_speed_threshold or (speed > _VELOCITY_EPS and rim_speed > speed * 1.05):
        _apply_rolling_regime(ball, table, dt)
        return

    max_reduction = table.sliding_friction_coefficient * GRAVITY_MPS2 * dt
    reduction = min(max_reduction, slip_speed)
    slip_hat = slip / slip_speed

    ball.velocity = ball.velocity - slip_hat * reduction

    if speed >= _VELOCITY_EPS:
        v_hat = ball.velocity / norm(ball.velocity) if norm(ball.velocity) >= _VELOCITY_EPS else vec2(1.0, 0.0)
        ball.omega += (reduction / ball.radius) * float(np.dot(slip_hat, v_hat))

    if table.spin_decay_rate > 0.0:
        ball.omega *= max(0.0, 1.0 - table.spin_decay_rate * dt)

    ball.position = ball.position + ball.velocity * dt


def _apply_rolling_regime(ball: Ball, table: TableConfig, dt: float) -> None:
    """Rolling resistance on translation and exponential spin decay."""
    apply_rolling_resistance(ball, table, dt)
    if table.spin_decay_rate > 0.0 and abs(ball.omega) > 0.0:
        decay = max(0.0, 1.0 - table.spin_decay_rate * dt)
        ball.omega *= decay
        if abs(ball.omega) < table.omega_stop_threshold:
            ball.omega = 0.0


def is_pure_rolling(ball: Ball, *, tolerance: float = 1e-6) -> bool:
    """Return True when center speed matches spin-induced rim speed (|v| ≈ |omega| r)."""
    speed = ball.speed
    rim = abs(ball.omega) * ball.radius
    if speed < _VELOCITY_EPS and rim < _VELOCITY_EPS:
        return True
    return abs(speed - rim) <= tolerance


def apply_stopping(ball: Ball, table: TableConfig) -> None:
    """Clamp translational and angular velocity when below stop thresholds."""
    if not ball.active:
        return
    if ball.speed < table.velocity_stop_threshold:
        ball.velocity = vec2(0.0, 0.0)
    if abs(ball.omega) < table.omega_stop_threshold:
        ball.omega = 0.0
    if ball.speed < table.velocity_stop_threshold and abs(ball.omega) < table.omega_stop_threshold:
        ball.stop()


def step_ball_spin(ball: Ball, table: TableConfig, dt: float) -> None:
    """Single-timestep update with spin-aware cloth friction."""
    if not ball.active:
        return
    if dt <= 0.0:
        raise ValueError("dt must be positive")

    apply_cloth_friction(ball, table, dt)
    apply_stopping(ball, table)
