import numpy as np
import pytest

from sim_core.physics.rack import create_break_setup
from sim_core.physics.shot import CueStrike
from sim_core.utils.vectors import vec2


def test_cue_strike_velocity_follows_aim_direction() -> None:
    strike = CueStrike(speed=3.0, aim_direction=vec2(-1.0, 0.2))
    setup = create_break_setup(cue_strike=strike)
    cue = setup.balls[setup.cue_ball_index]

    expected_direction = strike.aim_direction / np.linalg.norm(strike.aim_direction)
    observed_direction = cue.velocity / np.linalg.norm(cue.velocity)
    np.testing.assert_allclose(observed_direction, expected_direction, rtol=1e-7, atol=1e-7)
    assert np.linalg.norm(cue.velocity) == pytest.approx(strike.speed)


def test_cue_strike_parallel_offset_sets_omega_sign() -> None:
    follow = CueStrike(speed=2.0, aim_direction=vec2(-1.0, 0.0), hit_offset_parallel=0.5)
    draw = CueStrike(speed=2.0, aim_direction=vec2(-1.0, 0.0), hit_offset_parallel=-0.5)
    follow_setup = create_break_setup(cue_strike=follow)
    draw_setup = create_break_setup(cue_strike=draw)

    follow_ball = follow_setup.balls[follow_setup.cue_ball_index]
    draw_ball = draw_setup.balls[draw_setup.cue_ball_index]

    assert follow_ball.omega > 0.0
    assert draw_ball.omega < 0.0
    assert follow_ball.angular_velocity[2] > 0.0
    assert draw_ball.angular_velocity[2] < 0.0


def test_cue_strike_perpendicular_sets_side_spin() -> None:
    strike = CueStrike(
        speed=2.0,
        aim_direction=vec2(-1.0, 0.0),
        hit_offset_perpendicular=0.4,
    )
    setup = create_break_setup(cue_strike=strike)
    cue = setup.balls[setup.cue_ball_index]

    assert abs(cue.angular_velocity[1]) < 1e-9
    assert cue.angular_velocity[0] < -1.0
    assert cue.omega == 0.0
