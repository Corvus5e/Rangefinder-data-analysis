
// LeastSquares.cpp

#include <rda\function\LeastSquares.h>
#include <math.h>

using namespace function;

LeastSquares::LeastSquares(int exponent){
	if(exponent < 0)
		throw "exponent must be > 0 \n";

	this->additionalSummands = 2; // method requires two more summands for correct calculating
	this->exponent = exponent + this->additionalSummands;

	this->a = new double[this->exponent];
	this->beta = new double[this->exponent];
	this->alfa = new double[this->exponent];
}

int LeastSquares::approximate(){

	this->a[0] = 0;
	this->a[1] = this->getA(1);
	this->beta[0] = 0;
	this->beta[1] = 0;
	this->alfa[0]  = 0;
	this->alfa[1] = 0;

	for(int j=this->additionalSummands; j < this->exponent; j++){
		
		this->alfa[j] = this->getAlfa(j);
		this->beta[j] = this->getBeta(j);
		this->a[j] = this->getA(j);

	}

	return 0;
}

double LeastSquares::value(double x){
	return this->fi(x);
}

double LeastSquares::fi(double x){

	double result = 0.0;

	for(int i=0; i < this->exponent; i++){
		result += this->a[i]*this->q(i, x);
	}
	return result;
}

double LeastSquares::q(int j, double x){
	if(j == 0)
		return 0;
	if(j == 1)
		return 1;
	return x*this->q(j-1, x) - this->alfa[j]*this->q(j-1,x) - this->beta[j-1]*this->q(j-2, x); 
}

double LeastSquares::getA(int j){
	double up = 0.0;
	double down = 0.0;

	for(int i=this->begin; i <= this->end; i++){
		up += this->q(j, this->cloud->at(i).x)*this->cloud->at(i).y;
	}
	for(int i=this->begin; i <= this->end; i++){
		down += pow(this->q(j, this->cloud->at(i).x), 2.0);
	}

	return up/down;
}

double LeastSquares::getBeta(int j){

	double up = 0.0;
	double down = 0.0;

	for(int i=this->begin; i <= this->end; i++){
		up += this->cloud->at(i).x*this->q(j,this->cloud->at(i).x)*this->q(j-1, this->cloud->at(i).x);
	}

	for(int i=this->begin; i <= this->end; i++){
		down += pow(this->q(j-1, this->cloud->at(i).x), 2.0);
	}
	return up/down;
}

double LeastSquares::getAlfa(int j){
	double up = 0.0;
	double down = 0.0;

	for(int i=this->begin; i <= this->end; i++){
		up += this->cloud->at(i).x * pow(this->q(j-1, this->cloud->at(i).x), 2.0);
	}

	for(int i=this->begin; i <= this->end; i++){
		down += pow(this->q(j-1, this->cloud->at(i).x), 2.0);
	}

	return up/down;
}

LeastSquares::~LeastSquares(){
	delete[] a;
	delete[] alfa;
	delete[]  beta;
}