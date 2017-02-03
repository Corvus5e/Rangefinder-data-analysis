/*
 * Derivative.h
*/

#ifndef DERIVATIVE_H
#define DERIVATIVE_H

#include <rda\function\Function.h>

namespace function {

	class Derivative : public Function {

	public:

		Derivative(Function* function);

		double value(double argument);

	private:

		Function* func;

	};


}
#endif