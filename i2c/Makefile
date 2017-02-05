SOURCES=$(wildcard *.c)
CC=gcc
CFLAGS=-c -Wall -Wpedantic
LDFLAGS=
OBJECTS=$(SOURCES:.c=.o)
EXECUTABLE=test

$(EXECUTABLE): $(OBJECTS)
	$(CC) $(OBJECTS) -o $@ $(LDFLAGS)

$(OBJECTS): $(SOURCES)
	$(CC) $(CFLAGS) $*.c

clean:
	rm *.o $(EXECUTABLE)

 



