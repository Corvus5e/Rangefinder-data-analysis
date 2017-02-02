
#ifndef RDA_CONSOLE_H
#define RDA_CONSOLE_H

#include <string>
#include <map>

namespace rda {

	class Console {
	
	private :
	
		std::map<std::string, std::string> params_;		
	
	public:
	
		void readArgs(int argc, char* args[]);
	
		std::string& getParam(const std::string& param);
	};

}

#endif