
#ifndef RDA_CLOUD_PART_H
#define RDA_CLOUD_PART_H

#include <rda\common.h>
#include <rda\line.h>

namespace rda {

	class CloudPart {

	private:

		//CloudPart();

	protected:

		rda::CloudPtr cloud_;
		rda::Range range_;

	public: 		

		CloudPart(rda::CloudPtr cloud);

		CloudPart(rda::CloudPtr cloud, rda::Range range);

		rda::CloudPtr cloud();

		rda::Point& at(int index);

		rda::Point& first_point();

		rda::Point& last_point();

		rda::Range& range();

		int begin();

		int end();

		int size();

		// Line from first to last point
		rda::Line line(); 
	};
}

#endif
