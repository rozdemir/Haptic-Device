all:	
	g++ Source.cpp -o proje `pkg-config --cflags --libs opencv` -L/usr/local/lib  -lwiringPi -lwiringPiDev -lpthread -lm

