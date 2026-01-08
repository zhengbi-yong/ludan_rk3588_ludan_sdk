# LowState.msg
# Robot low-level state message
# Motor distribution: 10 + 7 + 7 + 6 = 30 motors total

# Header for timestamp
std_msgs/Header header

# Motor states (30 motors)
MotorState[30] motor_state
