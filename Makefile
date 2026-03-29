CC = cc
CFLAGS = -Wall -Wextra -O2 -I/usr/local/include
LDFLAGS = -L/usr/local/lib -lhackrf -lm -framework AudioToolbox

TARGET = bin/radio

.PHONY: all
all: $(TARGET)

$(TARGET): main.c
	mkdir -p bin
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

.PHONY: clean
clean:
	rm -f $(TARGET) main.o.json compile_commands.json
