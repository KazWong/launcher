#ifndef SERIAL_H
#define SERIAL_H

//    to check stm echo message, install gtkterm
//            sudo apt-get install gtkterm
//            set port and baudrate on menu item 'Configuration'-'Port'
//            port="/dev/ttyACM0"
//            baudrate=115200

#include <thread>
#include <chrono>

#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <fcntl.h>


class STM32_NUCLEO {

private:
	std::string port;
	std::fstream handler;

public:
	STM32_NUCLEO(std::string _port) : handler(NULL), port(_port) {
		if ( !Init() )
			throw "FAIL to open port";
	}
	
	~STM32_NUCLEO() {
		handler.close();
	}

	bool Init() {
		std::cout << port << std::endl;
		if ( !port.empty() )
			handler.open(port.c_str(), std::fstream::in | std::fstream::out);
		
		if ( handler.is_open() )
			return true;
		
		return false;
	}
	
	void Write(const char *msg) {
		//std::cout << "write: " << msg << std::endl;
		handler << msg;
		//handler.write(msg, 50);
		handler.flush();
	}
	
	void Read(char *msg, int size) {
		std::string tmp;
		
		//handler >> tmp;
		getline(handler, tmp);
		strcpy(msg, tmp.c_str());
		//std::cout << "read: " << msg << std::endl;
	}
};


class Named_Pipe {

private:
	std::string port;
	int handler;

public:
	Named_Pipe(std::string _port) : handler(-1), port(_port) {
		if ( !Init() )
			throw "FAIL to open port";
	}
	
	~Named_Pipe() {
		close(handler);
	}

	bool Init() {
		std::cout << port << std::endl;
		if ( !port.empty() )
			handler = open(port.c_str(), O_RDONLY);
		
		if (handler >= 0)
			return true;
		
		return false;
	}
	
	void Write(const char *msg) {
		write(handler, msg, sizeof(msg));
	}
	
	void Read(char *msg, int size) {
		read(handler, msg, size);
		//std::cout << "read: " << msg << std::endl;
	}
};

#endif // SERIAL_H

