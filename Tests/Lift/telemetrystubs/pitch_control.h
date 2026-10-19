#ifndef TEST_TELEMETRY_PITCH_H
#define TEST_TELEMETRY_PITCH_H
typedef struct {float target_angle_deg;} pitch_command_t;
typedef struct {float commanded_angle_deg,measured_angle_deg,commanded_velocity_deg_s,measured_velocity_deg_s;} pitch_status_t;
extern pitch_command_t g_pitch_control_command;
extern pitch_status_t g_pitch_control_status;
#endif
