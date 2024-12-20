uint64 timestamp # time since system start (microseconds)

float32 measured_yaw_rate      # [rad/s] Measured yaw rate
float32 adjusted_rate_setpoint # [rad/s] Rate setpoint that is being tracked (Applied slew rates)
float32 pid_yaw_rate_integral  # Integral of the PID for the closed loop yaw rate controller

# TOPICS rover_rate_status
