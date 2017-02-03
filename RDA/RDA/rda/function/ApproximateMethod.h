
// Interface ApproximateMethod

#ifndef APPROXIMATE_MATHOD_H
#define APPROXIMATE_MATHOD_H


#include <rda\function\Function.h>
#include <rda\common.h>


namespace function {

class ApproximateMethod : public Function
{
public:
	
	void init(rda::CloudPtr cloud, int begin_index, int end_index);

	// creates approximate function
	virtual int approximate() = 0;

 	virtual ~ApproximateMethod();

protected:

	rda::CloudPtr cloud;
	int begin;
	int end;

};

}

#endif