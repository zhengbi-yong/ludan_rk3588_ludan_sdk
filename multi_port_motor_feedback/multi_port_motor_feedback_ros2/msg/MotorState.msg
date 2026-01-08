# MotorState.msg
# Single motor state message

# Error code (DM motor format)
uint8 state

# Scaled measurements
int8  id                # Motor ID, 1-30
float32 position
float32 velocity
float32 effort          # Torque in Nm
int8  temp_mos
int8  temp_rotor
