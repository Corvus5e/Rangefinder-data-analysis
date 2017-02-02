
#ifndef RDA_LINE_H
#define RDA_LINE_H

#include <rda\common.h>
#include <rda\vector.h>

namespace rda {

	class Line {

		rda::CloudPtr cloud_; // cloud for two points

	public:

		Line(rda::Point& start, rda::Point& end);

		rda::Point& start();

		rda::Point& end();

		rda::Vector normVector();

		rda::Vector directionVector();

		rda::Point middlePoint();

		double length();

		// k coficient : y = kx + b
		double k();

		// b coficient : y = kx + b
		double b();

		static void intersection(Line l1, Line l2, rda::Point* intersection_point);		
	};	
}

#endif
