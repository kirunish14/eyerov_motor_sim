import math

from eyerov_motor_sim.motor_model import MotorModel

DT = 0.01


def run(model, target, seconds):
    out = []
    for _ in range(int(round(seconds / DT))):
        out.append(model.step(target, DT))
    return out


def test_starts_at_rest():
    assert MotorModel(0.4, 3000.0).rpm == 0.0


def test_reaches_63_percent_after_one_tau():
    m = MotorModel(tau=0.4, max_rpm=3000.0)
    run(m, 1000.0, 0.4)
    assert math.isclose(m.rpm, 1000.0 * (1 - math.exp(-1)), rel_tol=1e-6)


def test_settles_near_target():
    m = MotorModel(tau=0.4, max_rpm=3000.0)
    run(m, 1500.0, 4.0)
    assert math.isclose(m.rpm, 1500.0, rel_tol=0.01)


def test_response_is_monotonic_without_overshoot():
    m = MotorModel(tau=0.4, max_rpm=3000.0)
    samples = run(m, 1500.0, 3.0)
    assert all(b >= a for a, b in zip(samples, samples[1:]))
    assert max(samples) <= 1500.0


def test_saturates_at_max_rpm():
    m = MotorModel(tau=0.1, max_rpm=3000.0)
    samples = run(m, 99999.0, 3.0)
    assert max(samples) <= 3000.0


def test_reverse_direction_works():
    m = MotorModel(tau=0.1, max_rpm=3000.0)
    run(m, 1000.0, 1.0)
    run(m, -1000.0, 2.0)
    assert m.rpm < -900.0


def test_acceleration_limit_is_respected():
    m = MotorModel(tau=0.0, max_rpm=3000.0, max_accel_rpm_s=1000.0)
    prev = 0.0
    for _ in range(50):
        cur = m.step(3000.0, DT)
        assert cur - prev <= 1000.0 * DT + 1e-9
        prev = cur


def test_deadband_ignores_small_commands():
    m = MotorModel(tau=0.1, max_rpm=3000.0, deadband_rpm=50.0)
    run(m, 30.0, 1.0)
    assert m.rpm == 0.0


def test_zero_tau_is_instant():
    m = MotorModel(tau=0.0, max_rpm=3000.0)
    assert m.step(1200.0, DT) == 1200.0


def test_non_positive_dt_changes_nothing():
    m = MotorModel(tau=0.4, max_rpm=3000.0)
    assert m.step(1000.0, 0.0) == 0.0


def test_stable_with_very_large_dt():
    m = MotorModel(tau=0.1, max_rpm=3000.0)
    assert 0.0 <= m.step(1000.0, 100.0) <= 1000.0


def test_reset():
    m = MotorModel(tau=0.1, max_rpm=3000.0)
    run(m, 1000.0, 1.0)
    m.reset()
    assert m.rpm == 0.0
