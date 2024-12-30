#include "ros2_control_demo_example_2/ComDriver.h"

#include <libserial/SerialPort.h>
#include <libserial/SerialPortConstants.h>
#include <stdint.h>
#include <iostream>
#include <vector>

using namespace std;
using LibSerial::SerialPort ;
using LibSerial::BaudRate ;
using LibSerial::ReadTimeout ;

void ComDriver::init()
{
    serial_port.Open( "/dev/ttyACM0" ) ;
    serial_port.SetBaudRate( BaudRate::BAUD_9600 ) ;
}

Com_Status_t ComDriver::read_velocity()
{
    LibSerial::DataBuffer buffer;

    try
    {
        serial_port.WriteByte((unsigned char)0x04);

        // Read a byte from the serial port using SerialPort Read() methods.
        serial_port.Read(buffer, 3, 100) ;

    }
    catch (const ReadTimeout&)
    {
        std::cerr << "The Read() call has timed out." << std::endl ;
        return COM_FAIL;
    }

    if(buffer.size() == 3 && buffer[0] == 0x04) {
        pid_count_per_loop_left = (int8_t)buffer[1];
        pid_count_per_loop_right = (int8_t)buffer[2];
    }

    return COM_OK;
}

Com_Status_t ComDriver::write_velocity(int8_t pid_count_per_loop_left, int8_t pid_count_per_loop_right) 
{
    LibSerial::DataBuffer buffer;

    buffer.push_back(0x05);
    buffer.push_back((uint8_t)pid_count_per_loop_left);
    buffer.push_back((uint8_t)pid_count_per_loop_right);

    uint8_t return_code = 0;

    try
    {
        serial_port.Write(buffer);

        // Read a byte from the serial port using SerialPort Read() methods.
        serial_port.ReadByte(return_code, 100);
    }
    catch (const ReadTimeout&)
    {
        std::cerr << "The Read() call has timed out." << std::endl ;
        return COM_FAIL;
    }

    if(return_code != 0x05) {
        cout << "write fail!\n";
        return COM_FAIL;
    } 

    return COM_OK;
}

void ComDriver::print_velocity()
{
    cout << "left count per pid loop: " << (int)pid_count_per_loop_left << endl;
    cout << "right count per pid loop: " << (int)pid_count_per_loop_right << endl;
}

void ComDriver::close()
{
    // Close the Serial Port and Serial Stream.
    serial_port.Close() ;

    cout << "the com port has been closed!" << endl;
}

double convert_pid_loop_count_to_velocity(int8_t count_per_pid_loop)
{
    return (double)count_per_pid_loop / COUNT_PER_CYCLE * 3.1415 * 0.065 * INTERVAL_FREQUENCY;
}

int8_t convert_velocity_to_pid_loop_count(double velocity)
{
    return (int8_t)(velocity * COUNT_PER_CYCLE / 3.1415 / 0.065 / INTERVAL_FREQUENCY);
}