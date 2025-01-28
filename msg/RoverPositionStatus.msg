uint64 timestamp # time since system start (microseconds)

float32 measured_forward_speed          # [m/s] Measured speed in body x direction. Positiv: forwards, Negativ: backwards
float32 adjusted_forward_speed_setpoint # [m/s] Speed setpoint after applying slew rate
float32 measured_lateral_speed          # [m/s] Measured speed in body y direction. Positiv: right, Negativ: left
float32 adjusted_lateral_speed_setpoint # [m/s] Speed setpoint after applying slew rate
float32 pid_forward_throttle_integral   # Integral of the PID for the closed loop forward speed controller
float32 pid_lateral_throttle_integral   # Integral of the PID for the closed loop lateral speed controller

# TOPICS rover_position_status
