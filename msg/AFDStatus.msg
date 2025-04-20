# Current status of the FCUFLEX Active Force Device
float32 actual_force        # Current measured applied force
float32 actual_position     # Current measured carriage position
float32 command_force       # Current command force
float32 command_position    # Current command position
float32 payload_weight      # Current payload weight
float32 accel_gravity       # Current measured acceleration of gravity
int32 control_mode          # Control mode (0: Position, 1: Force, 3: Soft Touch)
bool tool_connected         # Tool connection status
bool weight_valid           # Weight valid status
bool pos_limit              # Positive position limit exceeded
bool neg_limit              # Negative position limit exceeded
bool soft_touch_active      # Soft touch feature is active
bool weight_in_position     # AFD is in position to perform weighing
bool weighing_in_progress   # Weighing procedure is in progress