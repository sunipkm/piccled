CC = gcc
EDCFLAGS = -Wall -Wextra -pedantic $(CFLAGS)
EDLDFLAGS = $(LDFLAGS)

# Source files
SRCS = $(wildcard *.c)
OBJS = $(SRCS:.c=.o)

TARGET = piccled.exe

.PHONY: all clean
all: $(TARGET)
$(TARGET): $(OBJS)
	$(CC) $(EDCFLAGS) -o $@ $^ $(EDLDFLAGS)

clean:
	rm -f $(OBJS) $(TARGET)

%.o: %.c
	$(CC) $(EDCFLAGS) -c $< -o $@