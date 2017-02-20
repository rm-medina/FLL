#ifndef __SERVOLIB_H_
#define __SERVOLIB_H_

#define SERVOLIB_MAX_SERVO_COUNT 6
#define SERVOLIB_NUM_SERVO_CMDS 5

#define SERVOLIB_MIN_PULSE_QUARTER_US 4000
#define SERVOLIB_MAX_PULSE_QUARTER_US 8000
#define SERVOLIB_MIN_STEP_QUARTER_US 100
#define SERVOLIB_DEF_STEP_QUARTER_US 200
#define HOME_POSITION_QUARTER_US 6000

enum servo_cmd{
	SERVOIO_RESET,
	SERVOIO_CHECK,
	SERVOIO_READ,
	SERVOIO_CONFIG,
	SERVOIO_WRITE,
	SERVOIO_NUM_CMDS,
};

struct servo_stats {
	int channel;
	int min_pos;
	int max_pos;
	int min_poserr;
	int max_poserr;
	int rt_err;
	enum servo_cmd lastcmd;
	int cmdstally[SERVOLIB_NUM_SERVO_CMDS];
};

struct servo_params {
	int channel;
	int min_position;
	int max_position;
	int poserr;
	int accel_limit;
	int speed_limit;
};

#ifdef __cplusplus
extern "C" {
#endif
int servoio_set_accel(int id, int channel, int value);
int servoio_set_speed(int id, int channel, int value);
int servoio_set_pulse(int id, int channel, int value);
int servoio_configure(int id, int channel, int pulse, int speed, int accel);
int servoio_get_position(int id, int channel);
int servoio_get_any_error(int id);
int servoio_all_go_home(int id);
int servoio_map_pulse(int pos, int id, int channel);
int servoio_map_coordinate(int coord, int min_coord, int max_coord);
int servoio_map_coordinate_with_restrictions(int coord, int min_coord,
					     int max_coord,
					     int min_pulse,
					     int max_pulse);

#ifdef __cplusplus
}
#endif

#endif
