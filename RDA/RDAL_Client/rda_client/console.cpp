
#include "console.h"
#include <iostream>

std::map<std::string, std::string> client::Console::params_ ;

void client::Console::readArgs(int argc, char* argv[]){

	if(argc > 1){
		for(int i = 1; i < argc - 1; i+=2){
			params_[argv[i]] = argv[i+1];
		}
	}
}

std::string client::Console::getParam(const std::string param){
	return params_[param];
}

void client::Console::printParams(){
	for(auto it = params_.begin(); it != params_.end(); it++){
		std::cout << it->first << ": " << it->second << std::endl;
	}
}