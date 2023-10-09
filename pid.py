class PID:
    def __init__(self, Kp, Ki, Kd, outMax, outMin, lim_int_min, lim_int_max, T, τ):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        # Derivative low-pass filter time constant
        self.τ = τ

        # Sample time (in seconds)
        self.T = T

        # Output limits
        self.outMax = outMax
        self.outMin = outMin

        # Integrator limits
        self.lim_int_min = lim_int_min
        self.lim_int_max = lim_int_max

        self.reset()

    def reset(self):
        self.integrator = 0
        self.previous_error = 0

        self.differentiator = 0
        self.previous_measurement = 0

        self.pid = 0

    def update(self, setPoint, measurement):
        error = setPoint - measurement

        proportional = self.Kp * error

        self.integrator += 0.5 * self.Ki * self.T * (error + self.previous_error)

        # Anti-wind-up via integrator clamping
        if self.integrator > self.lim_int_max:
            self.integrator = self.lim_int_max
        elif self.integrator < self.lim_int_min:
            self.integrator = self.lim_int_min

        self.differentiator = -(
            2.0 * self.Kd * (measurement - self.previous_measurement)
        ) + (2.0 * self.τ - self.T) * self.differentiator / (2.0 * self.τ + self.T)

        self.pid = proportional + self.integrator + self.differentiator

        # Output limits
        if self.pid > self.outMax:
            self.pid = self.outMax
        elif self.pid < self.outMin:
            self.pid = self.outMin

        self.previous_measurement = measurement
        self.previous_error = error

        return self.pid
