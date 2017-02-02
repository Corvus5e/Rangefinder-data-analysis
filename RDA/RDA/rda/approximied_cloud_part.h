
#ifndef APPROXIMIED_CLOUD_PART_H
#define APPROXIMIED_CLOUD_PART_H

#include <rda\cloud_part.h>

namespace rda {

	class ApproximiedCloudPart : public rda::CloudPart {

	private:
		
		rda::Line approx_line_;

	public:

		ApproximiedCloudPart(rda::CloudPtr cloud, rda::Range range, rda::Line appr_line);

		ApproximiedCloudPart(rda::CloudPart cloud_part, rda::Line appr_line);

		void set_approx_line(rda::Line appr_line);

		rda::Line approx_line();
	};
}

#endif