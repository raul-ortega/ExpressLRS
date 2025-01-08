#include <stdint.h>

/**
 * Cordic Algo to transform cartesian coordinates into polar coordinates
 */
void cartesian_to_polar_coordinates(int32_t north_dm, int32_t east_dm, int32_t *out_phi_deg, int32_t *out_radius_dm);

/*
 * Adapted from Ardupilot's AP_Frsky_SPort::prep_number()
 * This is a proprietary number format that must be known on a value basis between producer and consumer.
 */
uint16_t prep_number(int32_t number, uint8_t digits, uint8_t power);

/*
 * Adapted from Ardupilot's AP_Frsky_SPort_Passthrough::calc_ap_status()
 * This is the content of the 0x5001 AP_STATUS value.
 * Note that we don't have certain values via Mavlink.
 * - IMU Temperature
 * - simple/super simple mode flags
 * - specific failsafe flags
 * - fence flags
 */
uint32_t format_ap_status(uint8_t base_mode, uint32_t custom_mode, uint8_t system_status, uint16_t throttle);

/*
 * Adapted from Ardupilot's AP_Frsky_SPort_Passthrough::calc_gps_status()
 * This is the content of the 0x5002 GPS_STATUS value.
 */
uint32_t format_gps_status(uint8_t fix_type, int32_t alt_msl_mm, uint16_t eph, uint8_t satellites_visible);

/*
 * Adapted from Ardupilot's AP_Frsky_SPort_Passthrough::calc_batt()
 * This is the content of the 0x5003 BATT1 value.
 */
uint32_t format_batt1(uint16_t voltage_mv, uint16_t current_ca, uint32_t current_consumed);

/*
 * Adapted from Ardupilot's AP_Frsky_SPort_Passthrough::calc_home()
 * This is the content of the 0x5004 HOME value.
 */
uint32_t format_home(uint32_t distance_to_home_dm, uint32_t altitude_above_home_dm, uint8_t bearing_to_home_deg);

/*
 * Adapted from Ardupilot's AP_Frsky_SPort_Passthrough::calc_velandyaw()
 * This is the content of the 0x5005 Velocity and Yaw.
 */
uint32_t format_velandyaw(float climb_mps, float groundspeed_mps, int16_t heading);

/*
 * Adapted from Ardupilot's AP_Frsky_SPort_Passthrough::calc_attiandrng()
 * This is the content of the 0x5006 Attitude and RangeFinder.
 * We don't provide Rangefinder here.
 */
uint32_t format_attiandrng(float pitch_rad, float roll_rad);

/*
 * Adapted from Ardupilot's AP_Frsky_SPort_Passthrough::calc_terrain()
 * This is the content of the 0x500B terrain.
 */
uint32_t format_terrain(uint32_t altitude_terrain);

/*
 * Adapted from Ardupilot's AP_Frsky_SPort_Passthrough::calc_waypoint()
 * This is the content of the 0x500B terrain.
 */
uint32_t format_waypoint(uint8_t heading, uint16_t distance, uint16_t number);

