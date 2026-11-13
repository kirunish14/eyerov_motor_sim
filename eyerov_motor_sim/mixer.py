"""Pure-Python velocity mixer: Twist components -> target motor RPM.

No ROS imports here on purpose, so the maths can be unit-tested on its own.

Assumed thruster layout (documented in the README):
  * left / right : horizontal thrusters, track_width metres apart
  * vertical     : one heave thruster driven by linear.z
Sign convention follows ROS REP-103: +x forward, +z up,
+angular.z = counter-clockwise (turn left), so the right motor spins faster.
"""


def _clamp(value, limit):
    return max(-limit, min(limit, value))


def twist_to_target_rpm(linear_x, linear_z, angular_z,
                        track_width, max_speed, max_rpm):
    """Convert a velocity command into a target RPM for each motor.

    Args:
        linear_x:    forward speed command (m/s)
        linear_z:    vertical speed command (m/s)
        angular_z:   yaw rate command (rad/s)
        track_width: distance between left and right thrusters (m)
        max_speed:   speed (m/s) that corresponds to full motor RPM
        max_rpm:     RPM limit of a motor (positive number)

    Returns:
        dict {'left': rpm, 'right': rpm, 'vertical': rpm}
    """
    half_turn = angular_z * track_width / 2.0
    left = linear_x - half_turn
    right = linear_x + half_turn

    # If either side would exceed the limit, scale BOTH sides by the same
    # factor so the commanded turning ratio is preserved (desaturation).
    peak = max(abs(left), abs(right))
    if peak > max_speed:
        scale = max_speed / peak
        left *= scale
        right *= scale

    vertical = _clamp(linear_z, max_speed)

    k = max_rpm / max_speed
    return {
        'left': left * k,
        'right': right * k,
        'vertical': vertical * k,
    }
