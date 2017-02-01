
#ifndef RDA_EXCEPTION_H
#define RDA_EXCEPTION_H

#include <string>
#include <exception>

namespace rda {

	class RdaException : public std::exception {

	private:

		std:: string message_;

	public : 

		RdaException(std::string& message){
			this->message_ = message;
		}

		virtual const char* what() {
			return message_.c_str();
		}

	};

};

#endif
