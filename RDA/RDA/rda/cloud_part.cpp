
#include <rda\cloud_part.h>

rda::CloudPart::CloudPart(rda::CloudPtr cloud)
{
	this->cloud_ = cloud;
	this->range_ = rda::Range(0, cloud->size() - 1);
}

rda::CloudPart::CloudPart(rda::CloudPtr cloud, rda::Range range)
{
	this->cloud_ = cloud;
	this->range_ = range;
}

rda::CloudPtr rda::CloudPart::cloud()
{
	return this->cloud_;
}

rda::Point& rda::CloudPart::at(int i)
{
	return this->cloud_->at(i);
}

rda::Point& rda::CloudPart::first_point()
{
	return this->cloud_->at(this->range_.start);
}

rda::Point& rda::CloudPart::last_point()
{
	return this->cloud_->at(this->range_.end);
}

int rda::CloudPart::size()
{
	return ( this->range_.end - this->range_.start + 1 );
}

rda::Line rda::CloudPart::line()
{
	return rda::Line(this->first_point(), this->last_point());
}

rda::Range& rda::CloudPart::range()
{
	return this->range_;
}

int rda::CloudPart::begin()
{
	return this->range_.start;
}

int rda::CloudPart::end()
{
	return this->range_.end;
}