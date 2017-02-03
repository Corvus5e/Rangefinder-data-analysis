

#include <rda\line.h>

rda::Line::Line(rda::Point& start, rda::Point& end)
{
	rda::CloudPtr tmp(new rda::Cloud);
	this->cloud_ = tmp->makeShared();
	this->cloud_->push_back(start);
	this->cloud_->push_back(end);	
}

rda::Point& rda::Line::start()
{
	return this->cloud_->at(0);
}

rda::Point& rda::Line::end()
{
	return this->cloud_->at(1);
}

rda::Vector rda::Line::normVector()
{
	return rda::Vector( (this->start().y - this->end().y)/(this->start().x - this->end().x), -1.0);
}

rda::Vector rda::Line::directionVector()
{
	return rda::Vector( (this->start().x - this->end().x), (this->start().y - this->end().y)).unit();
}

rda::Point rda::Line::middlePoint()
{
	return rda::Point( (start().x + end().x) / 2.0, (start().y + end().y) / 2.0, (start().z + end().z) / 2.0 );
}

double rda::Line::k()
{
	double dx = this->end().x - this->start().x;
	if(dx != 0)
		return (this->end().y - this->start().y) / dx;
	return 0;	// !
}

double rda::Line::b()
{
	double dx = this->end().x - this->start().x;
	if(dx != 0)
		return (this->start().y * this->end().x - this->start().x * this->end().y) / dx;
	return 0;	// !
}

double rda::Line::length()
{
	return sqrt( pow((double)(this->end().x - this->start().x) ,2.0) + pow((double)(this->end().y - this->start().y), 2.0));
}

void rda::Line::intersection(rda::Line l1, rda::Line l2, rda::Point* p)
{
	if(!rda::Vector::isParallel(l1.directionVector(), l2.directionVector())){
		p->x = (l2.b() - l1.b())/(l1.k() - l2.k());
		p->y = l1.b() + l1.k() * p->x;
	}
	else{
		p = nullptr;
	}
}