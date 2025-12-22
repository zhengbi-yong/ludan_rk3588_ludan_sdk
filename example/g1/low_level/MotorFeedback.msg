# MotorFeedback.msg
# Motor feedback message from CAN bus

# Header for timestamp and frame ID
std_msgs/Header header

# Motor identification
int32 motor_id          # Motor ID (1-4)
int32 can_id            # Raw CAN ID

# Motor state
int32 mode              # Motor mode (0=disabled, 1=enabled, 2=error)
bool error             # Error flag
int32 error_code        # Error code if any

# Motor measurements
float32 position        # Position in radians
float32 velocity        # Velocity in rad/s
float32 torque          # Torque in Nm
float32 current         # Current in A

# Motor gains (echoed from command if available)
float32 kp             # Position gain
float32 kd             # Velocity gain

# Temperature
float32 temperature_mos     # MOS temperature in °C
float32 temperature_coil   # Coil temperature in °C

# Raw CAN data for debugging (optional)
uint8[] can_data        # Raw CAN frame data (8 bytes)