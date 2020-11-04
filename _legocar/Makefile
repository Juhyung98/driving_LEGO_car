
FLAGS=-I/root/src/gattlib/include
LDFLAGS=-Wl,-rpath,/root/src/gattlib/build/dbus
LIB=-lpthread -lgattlib -lwiringPi

test:
	gcc -o main main.c $(LIB) $(LDFLAGS) $(FLAGS)

all:
	gcc -o main main.c $(LIB)

debug:
	gcc -o main main.c $(LIB) -g
