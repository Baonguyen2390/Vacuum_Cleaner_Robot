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
    void print_velocity();
};

double convert_pid_loop_count_to_velocity(int8_t count_per_pid_loop);
int8_t convert_velocity_to_pid_loop_count(double velocity);