"""Cloth friction and slip velocity for spin-aware ball motion."""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

from sim_core.physics.ball import Ball
from sim_core.physics.integrator import apply_rolling_resistance
from sim_core.physics.table import TableConfig
from sim_core.utils.constants import GRAVITY_MPS2
from sim_core.utils.vectors import cross3, dot, norm, vec2, vec3

Vec2 = NDArray[np.float64]
Vec3 = NDArray[np.float64]

_VELOCITY_EPS = 1e-12


def table_contact_offset(radius: float) -> Vec3:
    """Vector from ball center to the cloth contact point at the table surface."""
    return vec3(0.0, 0.0, -radius)


def slip_velocity(ball: Ball) -> Vec2:
    """
    Tangential slip velocity at the cloth contact.

    Follow/draw uses motion-aligned rim slip (Phase 5-6). Side spin uses the
    bottom contact velocity from horizontal angular velocity components.
    """
    r = ball.radius
    omega = ball.angular_velocity
    r_bottom = table_contact_offset(r)
    side_slip = cross3(vec3(omega[0], omega[1], 0.0), r_bottom)[:2]

    speed = ball.speed
    if speed < _VELOCITY_EPS:
        return side_slip.copy()

    v_hat = ball.velocity / speed
    follow_draw = ball.velocity - omega[2] * r * v_hat
    return follow_draw + side_slip


def slip_speed(ball: Ball) -> float:
    """Return the magnitude of cloth slip velocity."""
    return norm(slip_velocity(ball))


def sphere_inertia(mass: float, radius: float) -> float:
    """Moment of inertia for a solid sphere about any axis through the center."""
    return 0.4 * mass * radius * radius


def apply_cloth_friction(ball: Ball, table: TableConfig, dt: float) -> None:
    """
    Update velocity and angular velocity from cloth friction.

    Sliding: kinetic friction opposite slip in the table plane, capped at
    mu_s * m * g, with torque tau = r_c x F. Rolling: Phase 1 rolling resistance
    plus optional spin decay on the full omega vector.
    """
    if not ball.active or dt <= 0.0:
        return

    slip = slip_velocity(ball)
    slip_speed = norm(slip)
    speed = ball.speed

    if (
        speed < table.velocity_stop_threshold
        and ball.spin_speed < table.omega_stop_threshold
    ):
        ball.stop()
        return

    if speed < table.velocity_stop_threshold:
        ball.velocity = vec2(0.0, 0.0)
        _apply_rolling_regime(ball, table, dt)
        return

    if speed < table.sliding_speed_threshold:
        _apply_rolling_regime(ball, table, dt)
        return

    if slip_speed < table.sliding_speed_threshold:
        _apply_rolling_regime(ball, table, dt)
        return

    max_reduction = table.sliding_friction_coefficient * GRAVITY_MPS2 * dt
    reduction = min(max_reduction, slip_speed)
    slip_hat = slip / slip_speed
    ball.velocity = ball.velocity - slip_hat * reduction

    if ball.speed >= _VELOCITY_EPS:
        v_hat = ball.velocity / max(ball.speed, _VELOCITY_EPS)
    else:
        v_hat = vec2(1.0, 0.0)
    slip_parallel = dot(slip, v_hat)
    ball.omega += (reduction / ball.radius) * float(np.sign(slip_parallel))

    horizontal_spin = ball.angular_velocity[0] ** 2 + ball.angular_velocity[1] ** 2
    if horizontal_spin > 1e-18:
        force = vec3(
            -slip_hat[0] * (ball.mass * reduction / dt),
            -slip_hat[1] * (ball.mass * reduction / dt),
            0.0,
        )
        torque = cross3(table_contact_offset(ball.radius), force)
        inertia = sphere_inertia(ball.mass, ball.radius)
        ball.angular_velocity = ball.angular_velocity + (torque / inertia) * dt

    if table.spin_decay_rate > 0.0:
        ball.angular_velocity *= max(0.0, 1.0 - table.spin_decay_rate * dt)

    _apply_swerve(ball, table, dt)
    ball.position = ball.position + ball.velocity * dt


def _apply_swerve(ball: Ball, table: TableConfig, dt: float) -> None:
    """Optional empirical side-force from horizontal spin components."""
    if table.swerve_coefficient <= 0.0 or ball.speed < _VELOCITY_EPS:
        return
    speed = ball.speed
    v_hat = ball.velocity / speed
    side_spin = ball.angular_velocity[0] * (-v_hat[1]) + ball.angular_velocity[1] * v_hat[0]
    if abs(side_spin) < 1e-12:
        return
    lateral = vec2(-v_hat[1], v_hat[0])
    acceleration = table.swerve_coefficient * side_spin * lateral
    ball.velocity = ball.velocity + acceleration * dt


def _apply_rolling_regime(ball: Ball, table: TableConfig, dt: float) -> None:
    """Rolling resistance on translation and exponential spin decay."""
    apply_rolling_resistance(ball, table, dt)
    if table.spin_decay_rate > 0.0 and ball.spin_speed > 0.0:
        decay = max(0.0, 1.0 - table.spin_decay_rate * dt)
        ball.angular_velocity *= decay
        if ball.spin_speed < table.omega_stop_threshold:
            ball.angular_velocity = vec3(0.0, 0.0, 0.0)


def is_pure_rolling(ball: Ball, *, tolerance: float = 1e-6) -> bool:
    """Return True when cloth slip speed is below tolerance."""
    return slip_speed(ball) <= tolerance


def apply_stopping(ball: Ball, table: TableConfig) -> None:
    """Clamp translational and angular velocity when below stop thresholds."""
    if not ball.active:
        return
    if ball.speed < table.velocity_stop_threshold:
        ball.velocity = vec2(0.0, 0.0)
    if ball.spin_speed < table.omega_stop_threshold:
        ball.angular_velocity = vec3(0.0, 0.0, 0.0)
    if ball.speed < table.velocity_stop_threshold and ball.spin_speed < table.omega_stop_threshold:
        ball.stop()


def step_ball_spin(ball: Ball, table: TableConfig, dt: float) -> None:
    """Single-timestep update with spin-aware cloth friction."""
    if not ball.active:
        return
    if dt <= 0.0:
        raise ValueError("dt must be positive")

    apply_cloth_friction(ball, table, dt)
    apply_stopping(ball, table)
