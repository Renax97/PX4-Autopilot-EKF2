uint64 timestamp # time since system start (microseconds)

float32 measured_yaw          # [rad/s] Measured yaw rate
float32 adjusted_yaw_setpoint # [rad/s] Yaw setpoint that is being tracked (Applied slew rates)
float32 pid_yaw_integral      # Integral of the PID for the closed loop yaw controller

# TOPICS rover_attitude_status
