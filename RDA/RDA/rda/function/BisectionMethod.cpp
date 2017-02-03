
/*
 * BisectionMethod.cpp
*/

#include <rda\function\BisectionMethod.h>
#include <math.h>

using namespace function;

double BisectionMethod::solve(Function& func, double rValue, double a, double b, double eps){

	double f0  = func.value(a);
	double f1 = func.value(b);
	/*if( ( (f0 - rValue )* ( f1 - rValue ) ) > 0)
		throw "no roots";*/

	int counter = 0;
	double root = (a + b)/2.0;
	double funcValue = func.value(root) - rValue;

	while( abs(funcValue) > eps  && ( counter < 20 ) ){

		root = (a + b)/2.0;
		funcValue = func.value(root) - rValue;
		if( ( (func.value(a) - rValue) * funcValue ) > 0){
			a = root;
		}
		else{
			b = root;
		}
		counter++;
	}

	return root;
}