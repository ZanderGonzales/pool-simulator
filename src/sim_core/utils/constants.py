"""Regulation-scale defaults (SI units)."""

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

# Ball-ball coefficient of restitution (1 = elastic, 0 = inelastic)
DEFAULT_COEFFICIENT_OF_RESTITUTION = 0.95
