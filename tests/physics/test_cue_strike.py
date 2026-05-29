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

    follow_omega = follow_setup.balls[follow_setup.cue_ball_index].omega
    draw_omega = draw_setup.balls[draw_setup.cue_ball_index].omega

    assert follow_omega > 0.0
    assert draw_omega < 0.0
