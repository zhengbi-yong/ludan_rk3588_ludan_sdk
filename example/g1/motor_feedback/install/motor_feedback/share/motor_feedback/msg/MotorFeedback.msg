# MotorFeedback.msg
# DM Motor feedback message from CAN bus
# Compatible with motor_controller_with_enable

# Header for timestamp and frame ID
std_msgs/Header header

# Motor identification
int32 motor_id          # Motor ID (1-30)
int32 can_id            # Raw CAN ID

# Motor mode/error (DM format)
int32 error             # Error code (4 bits from D[0] upper)
# 0x0: OK/Disabled
# 0x1: Enabled
# 0x8: Over-voltage
# 0x9: Under-voltage
# 0xA: Over-current
# 0xB: MOS Over-temp
# 0xC: Coil Over-temp
# 0xD: Comms Lost
# 0xE: Overload

# Raw measurements (DM format, unscaled)
int16 position          # Position raw value (16-bit signed)
int16 velocity          # Velocity raw value (12-bit signed)
int16 torque            # Torque raw value (12-bit signed)
int8 temp_mos           # MOS temperature
int8 temp_rotor         # Rotor temperature

# Scaled measurements (for convenience)
float32 position_rad    # Position in radians
float32 velocity_rad_s  # Velocity in rad/s
float32 torque_nm       # Torque in Nm

# Raw CAN data for debugging
uint8[8] can_data       # Raw CAN frame data (8 bytes)
