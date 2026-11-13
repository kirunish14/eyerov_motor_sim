import math

from eyerov_motor_sim.mixer import twist_to_target_rpm

ARGS = dict(track_width=0.3, max_speed=1.0, max_rpm=3000.0)


def mix(x=0.0, z=0.0, w=0.0):
    return twist_to_target_rpm(linear_x=x, linear_z=z, angular_z=w, **ARGS)


def test_zero_command_gives_zero_rpm():
    assert all(v == 0.0 for v in mix().values())


def test_forward_runs_both_motors_equally():
    out = mix(x=0.5)
    assert math.isclose(out['left'], 1500.0)
    assert math.isclose(out['right'], 1500.0)
    assert out['vertical'] == 0.0


def test_reverse_gives_negative_rpm():
    out = mix(x=-0.5)
    assert out['left'] < 0 and out['right'] < 0


def test_pure_rotation_counter_rotates_motors():
    out = mix(w=2.0)
    assert math.isclose(out['left'], -out['right'])
    assert out['right'] > 0  # +angular.z = turn left = right motor faster


def test_full_speed_hits_max_rpm_exactly():
    assert math.isclose(mix(x=1.0)['left'], 3000.0)


def test_over_limit_is_clamped_and_turn_ratio_preserved():
    out = mix(x=5.0, w=4.0)
    assert max(abs(out['left']), abs(out['right'])) <= 3000.0 + 1e-9
    ratio_cmd = (5.0 - 0.6) / (5.0 + 0.6)
    assert math.isclose(out['left'] / out['right'], ratio_cmd)


def test_vertical_follows_linear_z_and_clamps():
    assert math.isclose(mix(z=0.25)['vertical'], 750.0)
    assert math.isclose(mix(z=-9.0)['vertical'], -3000.0)
