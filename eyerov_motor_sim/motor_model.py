"""Pure-Python motor response model (the "firmware simulator" core).

A real motor cannot jump to a new speed instantly. This model combines:
  * command deadband      - tiny commands are ignored (stiction / ESC deadband)
  * saturation            - never exceeds +/- max_rpm
  * first-order lag       - rpm approaches target with time constant tau
  * acceleration limit    - RPM can change by at most max_accel_rpm_s * dt
"""
import math


class MotorModel:
    def __init__(self, tau, max_rpm, max_accel_rpm_s=0.0, deadband_rpm=0.0):
        """
        Args:
            tau:             lag time constant in seconds (0 = instant response)
            max_rpm:         saturation limit (positive)
            max_accel_rpm_s: max RPM change per second (0 = no limit)
            deadband_rpm:    |target| below this is treated as 0
        """
        self.tau = tau
        self.max_rpm = max_rpm
        self.max_accel_rpm_s = max_accel_rpm_s
        self.deadband_rpm = deadband_rpm
        self.rpm = 0.0

    def reset(self):
        self.rpm = 0.0

    def step(self, target_rpm, dt):
        """Advance the simulation by dt seconds and return the new RPM."""
        if dt <= 0.0:
            return self.rpm

        # 1. Condition the command: deadband, then saturation.
        if abs(target_rpm) < self.deadband_rpm:
            target_rpm = 0.0
        target_rpm = max(-self.max_rpm, min(self.max_rpm, target_rpm))

        # 2. First-order lag. Using the exact discretisation
        #    alpha = 1 - exp(-dt/tau) keeps the result correct (and stable)
        #    for any step size, unlike the Euler form dt/tau.
        if self.tau <= 0.0:
            desired_delta = target_rpm - self.rpm
        else:
            alpha = 1.0 - math.exp(-dt / self.tau)
            desired_delta = (target_rpm - self.rpm) * alpha

        # 3. Acceleration (slew-rate) limit.
        if self.max_accel_rpm_s > 0.0:
            limit = self.max_accel_rpm_s * dt
            desired_delta = max(-limit, min(limit, desired_delta))

        self.rpm += desired_delta
        return self.rpm
