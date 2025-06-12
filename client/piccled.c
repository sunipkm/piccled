/**
 * @file piccled.c
 * @author Sunip K. Mukherjee (sunipkmukherjee@gmail.com)
 * @brief 
 * @version 0.0.1
 * @date 2025-06-12
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>

#include "piccled.h"

int piccled_open(const char *port)
{
    int fd = open(port, O_RDWR);
    if (fd == -1)
    {
        return -1;
    }

    struct termios options;
    if (tcgetattr(fd, &options) < 0)
    {
        close(fd);
        return -1;
    }
    cfsetospeed(&options, B115200); // Set output baud rate
    cfsetispeed(&options, B115200); // Set input baud rate

    options.c_cflag &= ~PARENB; // No parity
    options.c_cflag &= ~CSTOPB; // One stop bit
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;            // 8 data bits
    options.c_cflag |= CREAD | CLOCAL; // Enable receiver, ignore modem control lines

    options.c_lflag &= ~ICANON; // Set raw mode
    options.c_lflag &= ~(ECHO | ECHOE | ISIG);

    options.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable flow control
    options.c_iflag &= ~(ICRNL | INLCR | IGNCR);

    options.c_oflag &= ~OPOST; // Disable output processing

    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 10; // Set timeout to 1 second (10 deciseconds)
    if (tcsetattr(fd, TCSANOW, &options) < 0)
    {
        close(fd);
        return -1;
    }
    return fd;
}

int piccled_set(int fd, uint8_t value)
{
    if (fd <= STDERR_FILENO)
    {
        return -1; // Invalid file descriptor
    }
    tcflush(fd, TCIFLUSH);
    char cmd[16] = {0x0, };
    if (value > 100)
    {
        value = 100; // Cap value at 100
    }
    size_t ret = snprintf(cmd, sizeof(cmd), "%d", value);
    if (ret >= sizeof(cmd))
    {
        return -1; // Buffer overflow, which is impossible
    }
    ssize_t bytes_written = write(fd, cmd, ret);
    if (bytes_written < 0)
    {
        return -1; // Write error
    }
    else if (bytes_written != (ssize_t) ret)
    {
        return -1; // Not all bytes written
    }
    bytes_written = read(fd, cmd, sizeof(cmd) - 1);
    if (bytes_written < 0)
    {
        return -1; // Read error
    }
    cmd[bytes_written] = '\0'; // Null-terminate the string
    if (strncmp(cmd, "OK\r\n", bytes_written > 2 ? 2 : bytes_written) != 0)
    {
        return -1; // Unexpected response
    }
    return 0; // Success
}