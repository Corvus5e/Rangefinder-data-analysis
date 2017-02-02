
#include <rda\approximied_cloud_part.h>

using namespace rda;

ApproximiedCloudPart::ApproximiedCloudPart(rda::CloudPtr cloud, rda::Range range, rda::Line appr_line) : CloudPart(cloud, range), approx_line_(appr_line) { }

ApproximiedCloudPart::ApproximiedCloudPart(rda::CloudPart cloud_part, rda::Line appr_line) : CloudPart(cloud_part), approx_line_(appr_line) { }

void ApproximiedCloudPart::set_approx_line(rda::Line appr_line)
{
	this->approx_line_ = appr_line;
}

Line ApproximiedCloudPart::approx_line()
{
	return this->approx_line_;
}