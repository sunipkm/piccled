#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include "piccled.h"

int main(int argc, char *argv[])
{
    if (argc != 3)
    {
        printf("Usage: %s <port> <value>\n", argv[0]);
        return 1;
    }
    const char *port = argv[1];
    uint8_t value = (uint8_t)atoi(argv[2]);
    int fd = piccled_open(port);
    if (fd < 0)
    {
        printf("Failed to open port %s: %s\n", port, strerror(errno));
        return 1;
    }
    int ret = piccled_set(fd, value);
    if (ret < 0)
    {
        printf("Failed to set value %d on port %s\n", value, port);
    }
    else
    {
        printf("Successfully set value %d on port %s\n", value, port);
    }
    close(fd);
    return 0;
}