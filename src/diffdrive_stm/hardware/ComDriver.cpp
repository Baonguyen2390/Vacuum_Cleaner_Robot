#include "diffdrive_stm/ComDriver.h"

#include <libserial/SerialPort.h>
#include <libserial/SerialPortConstants.h>
#include <stdint.h>
#include <iostream>
#include <vector>

using namespace std;
using LibSerial::SerialPort ;
using LibSerial::BaudRate ;
using LibSerial::ReadTimeout ;
using LibSerial::OpenFailed ;

void ComDriver::init()
{
    try
    {
        serial_port.Open( "/dev/ttyACM0" ) ;
        serial_port.SetBaudRate( BaudRate::BAUD_9600 ) ;
    }
    catch (const OpenFailed&)
    {
        std::cerr << "The serial ports did not open correctly." << std::endl ;
    }

    std::cout << "open port successfully!" << std::endl;
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
        std::cerr << "The Read() call has timed out in read_velocity" << std::endl ;
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
        std::cerr << "The Read() call has timed out in write_velocity" << std::endl ;
        return COM_FAIL;
    }

    if(return_code != 0x05) {
        cout << "write fail!\n";
        return COM_FAIL;
    } 

    return COM_OK;
}

Com_Status_t ComDriver::set_parameter(PIDLoop_Attr_t pid_parameter)
{
    LibSerial::DataBuffer buffer;
    buffer.push_back(COM_SERVICE_SET_PARAMETER);

    for(size_t i = 0; i < sizeof(PIDLoop_Attr_t); i++)
    {
        buffer.push_back(((uint8_t*)&pid_parameter)[i]);
    }

    uint8_t return_code = 0;

    try
    {
        serial_port.Write(buffer);

        // Read a byte from the serial port using SerialPort Read() methods.
        serial_port.ReadByte(return_code, 100);
    }
    catch (const ReadTimeout&)
    {
        std::cerr << "The Read() call has timed out in set_parameter" << std::endl ;
        return COM_FAIL;
    }

    if(return_code != COM_SERVICE_SET_PARAMETER) {
        cout << "fail to set parameters!\n";
        return COM_FAIL;
    } 

    return COM_OK;
}

Com_Status_t ComDriver::set_system_mode(com_system_state_t state)
{
    LibSerial::DataBuffer buffer;
    uint8_t return_code = 0;

    buffer.push_back(COM_SERVICE_SET_SYSTEM_MODE);
    buffer.push_back((uint8_t)state);

    try
    {
        serial_port.Write(buffer);

        // Read a byte from the serial port using SerialPort Read() methods.
        serial_port.ReadByte(return_code, 100);
    }
    catch (const ReadTimeout&)
    {
        std::cerr << "The Read() call has timed out in set system mode" << std::endl ;
        return COM_FAIL;
    }

    if(return_code != COM_SERVICE_SET_SYSTEM_MODE) {
        std::cerr << "fail to set system mode!\n";
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
