#include <iostream>
#include <string.h> // strlen()
#include <unistd.h> // usleep()
#include "SerialController.hpp"

/*
	In this setup, RX is tied directly to TX
	the physical serial controller acts as a 
	simple echo device
*/

using namespace std;

#define BUF_LENGTH 128

int main(int argc, char* argv[]) {
	// quick error check
	if(argc != 2) {
		cerr << "USAGE:\n    " << argv[0] << " <device port>\n";
		return -1;
	}

	SerialController sc(argv[1]);

	// setup basic 8n1 (9600) serial protocol
	sc.set_WordSize(WordSize_8);
	sc.set_Parity(Parity_NONE);
	sc.set_StopBits(StopBits_1);
	sc.set_BaudRate(BaudRate_9600);

	// start serial communications
	sc.start();

	// store incoming chars here
	char buf[BUF_LENGTH];

	// infinite loop
	for(;;) {
		// clear the buffer
		bzero(buf, BUF_LENGTH);

		// write and then read a string of chars
		char* hw = "Hello World";
		sc.writeBuffer(hw, strlen(hw));
		sc.readBuffer(buf, BUF_LENGTH);

		// print the received information
		cout << buf << endl;

		// sleep for ~100ms
		usleep(100000);
	}

	// pointless return statement
	return 0;
}