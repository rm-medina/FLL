#ifndef __SERVOLIB_H_
#define __SERVOLIB_H_

#define SERVOLIB_MAX_SERVO_COUNT 6

struct servo_params {
	int channel;
	int position;
	int accel_limit;
	int speed_limit;
};

enum servo_cmd{
	SERVOIO_RESET,
	SERVOIO_CHECK,
	SERVOIO_READ,
	SERVOIO_CONFIG,
	SERVOIO_WRITE,
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

#ifdef __cplusplus
}
#endif

#endif
