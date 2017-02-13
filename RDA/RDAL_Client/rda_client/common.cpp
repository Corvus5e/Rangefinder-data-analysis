
#include "common.h"

void client::split(std::string str, char* delimiter, std::vector<std::string>& parts){

	char* s = (char*)str.c_str();
	const char* d = delimiter;
 
	char *next_token1 = 0;
	char *token1 = 0;
	token1 = strtok_s(s, d, &next_token1);
	while ((token1 != 0)){
		if (token1 != 0)
		{
			parts.push_back(token1);
			token1 = strtok_s(0, d, &next_token1);
		}
	}	
}


client::RDAException::RDAException(const char* message){
	this->message = message;
}

const char* client::RDAException::what() {
	return this->message;
}
