
#ifndef RDA_CLIENT_COMMON_H
#define RDA_CLIENT_COMMON_H

#include <string>
#include <vector>
#include <exception>

namespace client {

	class RDAException : public std::exception {

	private :

		const char* message;

	public: 

		RDAException(const char* message);	

		virtual const char* what();
	};

	void split(std::string str, char* delimiter, std::vector<std::string>& parts);
}

#endif
