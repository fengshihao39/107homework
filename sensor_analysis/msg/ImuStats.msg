# Header uses the latest sensor timestamp, not the playback wall clock.
std_msgs/Header header
uint32 window_samples
float64 window_seconds
float64 sample_rate_hz
float64 accel_norm_mean_mps2
float64 accel_norm_std_mps2
float64 gyro_norm_rms_radps
float64 gravity_norm_deviation_mps2
# Heuristic only; uniform straight motion can also satisfy these thresholds.
bool stationary_candidate
uint64 received_samples
uint64 rejected_samples
uint64 nonmonotonic_samples
uint64 gap_events
uint64 session_resets
