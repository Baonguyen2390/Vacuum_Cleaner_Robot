#ifndef _COM_DRIVE_H_
#define _COM_DRIVE_H_

#include <libserial/SerialPort.h>
#include <libserial/SerialPortConstants.h>
#include <stdint.h>
#include <iostream>
#include <vector>

#define COUNT_PER_CYCLE 11
#define INTERVAL_FREQUENCY 30

using namespace std;
using LibSerial::SerialPort ;
using LibSerial::BaudRate ;
using LibSerial::ReadTimeout ;

typedef enum {
    COM_FAIL = 0,
    COM_OK,
} Com_Status_t;

typedef enum com_system_state_t
{
	SYSTEM_STATE_RESET = 0,
	SYSTEM_STATE_RUNNING = 1,
} com_system_state_t;

typedef enum {
    COM_SERVICE_READ_ENCODER = 0x04,
    COM_SERVICE_VELOCITY_WRITE = 0x05,
    COM_SERVICE_SET_PARAMETER = 0x06,
    COM_SERVICE_SET_SYSTEM_MODE = 0x07,
    COM_SERVICE_NEGATIVE_RESPONE = 0x7F,
} ComService_t;

typedef struct PIDLoop_Attr_t
{
	int max_count_per_loop;
	int frequency;
	double p;
	double i;
	double d;
} PIDLoop_Attr_t;

class ComDriver {
public:
    SerialPort serial_port ;
    size_t timeout_milliseconds = 100 ;
    int8_t pid_count_per_loop_left = 0;
    int8_t pid_count_per_loop_right = 0;

    void init();
    void close();
    Com_Status_t read_velocity();
    Com_Status_t write_velocity(int8_t pid_count_per_loop_left, int8_t pid_count_per_loop_right);
    Com_Status_t set_parameter(PIDLoop_Attr_t pid_parameter);
    Com_Status_t set_system_mode(com_system_state_t state);
    void print_velocity();
};

double convert_pid_loop_count_to_velocity(int8_t count_per_pid_loop);
int8_t convert_velocity_to_pid_loop_count(double velocity);

#endif