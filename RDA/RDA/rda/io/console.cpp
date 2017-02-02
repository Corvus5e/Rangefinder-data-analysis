
#include <rda\io\console.h>
#include <iostream>

void rda::Console::readArgs(int argc, char* argv[])
{
	if(argc > 1){
		for(int i = 1; i < argc - 1; i+=2){
			params_[argv[i]] = argv[i+1];
		}
	}
}

std::string& rda::Console::getParam(const std::string& param)
{
	return params_[param];
}