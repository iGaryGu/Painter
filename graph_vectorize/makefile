TARGET=vector
CC=gcc
SRC=main.c
CFLAGS=-g -Wall

run:$(TARGET)
	@./$(TARGET)

$(TARGET):$(SRC)
	$(CC) -o $@ $(CFLAGS) $^
clean:
	rm -f $(TARGET)

