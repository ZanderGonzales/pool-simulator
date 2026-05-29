"""Regulation-scale defaults (SI units)."""

from __future__ import annotations

import math

INCH_TO_M = 0.0254
GRAVITY_MPS2 = 9.80665

# Regulation ball: 2.25 in diameter -> 0.05715 m radius
BALL_RADIUS_M = 2.25 * INCH_TO_M / 2.0
BALL_MASS_KG = 0.17

# 9 ft table playing surface (approx. 100 in x 50 in)
TABLE_WIDTH_M = 100.0 * INCH_TO_M
TABLE_HEIGHT_M = 50.0 * INCH_TO_M

# Rolling resistance coefficient for a pool ball on cloth (dimensionless).
DEFAULT_ROLLING_RESISTANCE_COEFFICIENT = 0.02

# Velocity below which a ball is considered stopped (m/s)
DEFAULT_VELOCITY_STOP_THRESHOLD = 1e-4

# Angular velocity below which spin is considered stopped (rad/s)
DEFAULT_OMEGA_STOP_THRESHOLD = 1e-3

# Cloth sliding (kinetic) friction coefficient for slip-based model
DEFAULT_SLIDING_FRICTION_COEFFICIENT = 0.2

# Coulomb cap on ball-ball tangential impulse relative to normal impulse
DEFAULT_BALL_BALL_FRICTION = 0.15

# Exponential spin decay rate in rolling regime (1/s)
DEFAULT_SPIN_DECAY_RATE = 2.0

# Slip speed below which rolling resistance replaces sliding friction (m/s)
SLIDING_SPEED_THRESHOLD = 0.01

# Ball-ball coefficient of restitution (1 = elastic, 0 = inelastic)
DEFAULT_COEFFICIENT_OF_RESTITUTION = 0.95

# Default rack placement along the long table axis (fraction of width).
DEFAULT_FOOT_SPOT_X_FRACTION = 0.25
DEFAULT_HEAD_SPOT_X_FRACTION = 0.75

# Vertical spacing between triangle rows (touching balls).
ROW_SPACING_M = math.sqrt(3.0) * BALL_RADIUS_M
