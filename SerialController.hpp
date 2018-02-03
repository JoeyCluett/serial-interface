/**
This is free and unencumbered software released into the public domain.

Anyone is free to copy, modify, publish, use, compile, sell, or
distribute this software, either in source code form or as a compiled
binary, for any purpose, commercial or non-commercial, and by any
means.

In jurisdictions that recognize copyright laws, the author or authors
of this software dedicate any and all copyright interest in the
software to the public domain. We make this dedication for the benefit
of the public at large and to the detriment of our heirs and
successors. We intend this dedication to be an overt act of
relinquishment in perpetuity of all present and future rights to this
software under copyright law.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE SOFTWARE.

For more information, please refer to <http://unlicense.org>
*/

#ifndef __JJC__SERIALCONTROLLER__H__
#define __JJC__SERIALCONTROLLER__H__

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <vector>
#include <stdlib.h>

#define SC SerialController

enum Parity {
    Parity_ODD,
    Parity_EVEN,
    Parity_NONE
};

enum WordSize {
    WordSize_5,
    WordSize_6,
    WordSize_7, 
    WordSize_8
};

enum StopBits {
    StopBits_1, 
    StopBits_2
};

enum BaudRate {
    BaudRate_9600    = B9600,
    BaudRate_19200   = B19200,
    BaudRate_57600   = B57600,
    BaudRate_115200  = B115200
};

class SerialController {
private:
    termios tty;
    int fd = -1;
    bool serialPortSet = false;
    void parityEnable(void);

public:
    // constructor that does nothing
    SerialController(void);

    // constructor with serial port as argument
    SerialController(const char* serialPort);

    // specify a serial device to use
    void set_SerialPort(const char* serialPort);

    // try to read a certain number of bytes from the serial port
    int readBuffer(char* buffer, int bufSize);

    // try write a certain number of bytes from the serial port
    int writeBuffer(char* buffer, int bufSize);

    // block until operations are complete
    void readChunk(char* buffer, int bufSize);

    // guarantee write bufSize bytes
    void writeChunk(char* buffer, int bufSize);

    // set the read/write speeds for the serial port
    void set_BaudRate(BaudRate baudrate);

    // set odd/even/no parity for this serial port
    void set_Parity(Parity parity);

    // set either 7 or 8 bit word for this serial port
    void set_WordSize(WordSize wordsize);

    // set either 1 or 2 stop bits for this port
    void set_StopBits(StopBits stopbits);

    // start serial communications
    void start(void);

    // return the file descriptor associated with this port
    int get_FileDescriptor(void);

    // update external buffer of data
    int updateBuffer(std::vector<char>& vec);
};

SC::SerialController(void) {
    ;
}

SC::SerialController(const char* serialPort) {
    set_SerialPort(serialPort);
}

int SC::updateBuffer(std::vector<char>& vec) {
    char buf[1024];
    int i = read(fd, buf, 1024);

    for(int j = 0; j < i; j++)
        vec.push_back(buf[j]);

    return i;
}

void SC::set_SerialPort(const char* serialPort) {
    memset(&tty, 0, sizeof tty);
    fd = open(serialPort, O_RDWR | O_NOCTTY | O_NDELAY); // pretty standard flags
    if(fd < 0) {
        int e = errno;
        std::cerr << "Error opening file" << std::endl;
        std::cerr << "    Error code: " << e << std::endl;
        std::cerr << " -- " << strerror(e) << std::endl;
        exit(-1);
    }

    if(tcgetattr(fd, &tty) < 0) {
        int e = errno;
        std::cerr << "Error retrieving attributes" << std::endl;
        std::cerr << "    Error code: " << e << std::endl;
        std::cerr << " -- " << strerror(e) << std::endl;
        exit(-1);
    }

    serialPortSet = true;
}

void SC::parityEnable(void) {
    tty.c_cflag |= PARENB;
}

int SC::writeBuffer(char* buffer, int bufSize) {
    return write(fd, buffer, bufSize);
}

int SC::readBuffer(char* buffer, int bufSize) {
    return read(fd, buffer, bufSize);
}

void SC::readChunk(char* buffer, int bufSize) {
    int bytes_read = 0;
    while(bytes_read < bufSize)
        bytes_read += read(fd, buffer+bytes_read, bufSize-bytes_read);
}

void SC::writeChunk(char* buffer, int bufSize) {
    int bytes_writ = 0;
    while(bytes_writ < bufSize)
        bytes_writ += write(fd, buffer+bytes_writ, bufSize-bytes_writ);
}

void SC::set_BaudRate(BaudRate baudrate) {
    cfsetispeed(&tty, baudrate);
    cfsetospeed(&tty, baudrate);
}

void SC::set_Parity(Parity parity) {
    switch(parity) {
        case Parity_EVEN:
            tty.c_cflag |= PARENB;
            tty.c_cflag &= ~PARODD;
            break;
        case Parity_ODD:
            tty.c_cflag |= PARENB;
            tty.c_cflag |= PARODD;
            break;
        case Parity_NONE: // disable the parity bit
            tty.c_cflag &= ~PARENB;
            break;
        default:
            std::cerr << "Invalid parity argument" << std::endl;
            exit(-1); // all error comditions return -1
    }
}

void SC::set_StopBits(StopBits stopbits) {
    switch(stopbits) {
        case StopBits_1:
            tty.c_cflag &= ~CSTOPB;
            break;
        case StopBits_2:
            tty.c_cflag |= CSTOPB;
            break;
        default:
            std::cerr << "Invalid stop bit argument" << std::endl;
            exit(-1);
    }
}

void SC::set_WordSize(WordSize wordsize) {
    switch(wordsize) {
        case WordSize_5:
            tty.c_cflag &= ~CSIZE;
            tty.c_cflag |= CS5;
            break;
        case WordSize_6:
            tty.c_cflag &= ~CSIZE;
            tty.c_cflag |= CS6;
            break;
        case WordSize_7:
            tty.c_cflag &= ~CSIZE;
            tty.c_cflag |= CS7;
            break;
        case WordSize_8:
            tty.c_cflag &= ~CSIZE;
            tty.c_cflag |= CS8;
            break;
        default:
            std::cerr << "Invalid word size argument" << std::endl;
            exit(-1);
    }
}

void SerialController::start(void) {
    if(serialPortSet == false) {
        std::cerr << "Serial port has not been opened" << std::endl;
        exit(-1);
    }

    if(tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "Error starting serial communications" << std::endl;
        std::cerr << "    Error code: " << errno << std::endl;
        exit(-1);
    }
}

int SerialController::get_FileDescriptor(void) {
    return fd;
}

#endif // __JJC__SERIALCONTROLLER__H__


