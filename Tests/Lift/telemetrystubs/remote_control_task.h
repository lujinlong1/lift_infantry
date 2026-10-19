#ifndef TEST_TELEMETRY_REMOTE_H
#define TEST_TELEMETRY_REMOTE_H
typedef struct {float pitch_raw_rate_deg_s,pitch_filtered_rate_deg_s,pitch_ramped_rate_deg_s;} remote_mapping_t;
extern remote_mapping_t g_remote_mapping_status;
#endif
