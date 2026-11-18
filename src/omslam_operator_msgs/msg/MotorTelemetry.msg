# Empty arrays and fields absent from valid_fields mean not measured/provided.
# sim provides measured joint fields; mock provides encoder-derived fields.
# serial can additionally expose MCU feedback. target_rpm is command-derived
# unless the source reports the actual MCU target. counters preserve sub-sample
# stamp and age; valid entries are named counters.<key> in valid_fields.
std_msgs/Header header
builtin_interfaces/Time observation_stamp
string backend
string source
string[] valid_fields
float64 feedback_age_s
uint32[] raw_counts
int64[] counts
int64[] delta_counts
uint8[] ab
uint32[] invalid_transitions
float64[] position_rad
float64[] velocity_rad_s
float64[] encoder_rpm
float64[] target_rpm
float64[] measured_rpm
int16[] pwm
uint32 mcu_stamp_ms
bool armed
bool pid_mode
uint16 fault
string fault_reason
uint32 sequence
uint32 applied_sequence
uint32 applied_tick
uint32 target_age_ms
string mode
string[] wheel_states
bool[] settled
bool[] limited
uint32[] hold_reference
int32[] position_error
uint32 ack_sequence
string ack_status
string ack_reason
float64 ack_age_s
uint64 host_decoder_errors
diagnostic_msgs/KeyValue[] counters
