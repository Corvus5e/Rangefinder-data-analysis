
/*
 * Derivative.cpp
*/

#include <rda\function\Derivative.h>

using namespace function;

Derivative::Derivative(Function* function){
	this->func = function;
}

double Derivative::value(double x){
	double delta = 0.00001;
	return ((*this->func).value(x + delta) - (*this->func).value(x)) / delta;

}