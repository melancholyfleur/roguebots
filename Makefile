all:
	gcc `pkg-config --cflags playerc` -o player `pkg-config --libs playerc` player.c

