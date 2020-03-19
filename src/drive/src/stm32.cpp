#include <thread>
#include <chrono>

#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <string>

class STM32_NUCLEO {

private:
	std::string port;
	std::fstream handler;

public:
	STM32_NUCLEO(std::string _port) : handler(NULL), port(_port) {
		if ( !Init(port) )
			throw "FAIL to open port";
	}
	
	~STM32_NUCLEO() {
		handler.close();
	}

	bool Init(const std::string _port) {
		if (!_port.empty())
			handler.open(_port.c_str(), std::fstream::in | std::fstream::out);
		
		return true;
	}
	
	void Write(const char *msg) {
		handler << msg << std::flush;
	}
	
	void Read(std::string &msg, int size) {
		getline(handler, msg);
	}
	
};

int main(int argc, char** argv) {
	std::string s;
	STM32_NUCLEO encoder("/dev/ttyACM0");
	
	encoder.Write("SET 2 1\r\n");
	
	for (int i=0; i<10;i++) {
		encoder.Read(s, 50);
		std::cout << s << std::endl;
	}
}
