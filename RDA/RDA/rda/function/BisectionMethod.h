/*
 * BisectionMethod.h
 * BisectionMethod - class for finding root of function
*/

#ifndef BISECTION_METHOD_H
#define BISECTION_METHOD_H

#include <rda\function\EquationSolver.h>

namespace function {

class BisectionMethod : public EquationSolver {

public:

	double solve(Function& func, double rValue, double intervalFrom, double intervalTo, double eps = 0.1);
};

}

#endif