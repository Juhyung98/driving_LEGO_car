
FLAGS=-I/root/src/gattlib/include
LDFLAGS=-Wl,-rpath,/root/src/gattlib/build/dbus
LIB=-lpthread -lgattlib

all:
	gcc -o main main.c $(LIB) $(LDFLAGS) $(FLAGS)

test:
	gcc -o main main.c $(LIB)

