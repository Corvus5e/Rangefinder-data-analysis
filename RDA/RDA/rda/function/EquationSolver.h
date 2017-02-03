/*
 * EquationSolver.h
 * 
 * Interface for methods of solutions of equations 
*/

#ifndef EQUATION_SOLVER_H
#define EQUATION_SOLVER_H

#include "Function.h"

namespace function {

class EquationSolver {

public:
	// func = rValue => func - rValue = 0 (throws char*)
	virtual double solve(Function& func, double rValue, double interalFrom, double intervalTo, double eps = 0.1) = 0; 
};

}

#endif