CC = gcc
INCS = -I../Fusion/Fusion
LIBS = -lhidapi-libusb -lncurses -lm -lglfw -lGL -lGLEW -L../Fusion/Fusion/ -lFusion
TARGET = inspector

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(TARGET).c
	$(CC) $(INCS) -o $@ $^ $(LIBS)

clean:
	$(RM) $(TARGET)
