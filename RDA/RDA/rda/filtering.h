
#ifndef RDA_FILTERING_H
#define RDA_FILTERING_H

#include <rda\common.h>

namespace rda {

	void statisticalFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ> &filtered_cloud, int neighbour, double thresh );
	
	void medianFilter(std::vector<double>& values, int wsize, std::vector<double>& output);

	void reduceMedianFilter(std::vector<double>& values, rda::Range bounds, int wsize, std::vector<int>& indexes);

	void reduceMedianFilter(std::vector<double>& values, std::vector<int>& v_indexes, int wsize, std::vector<int>& indexes);

	// final window size is 2*window_size + 1
	void kuwaharaFilter(const std::vector<double>& values, int window_size, std::vector<double>& output);

	void statisticalDistanceFilter(std::vector<double>& distances, int neighbours_number, double threshold, std::vector<int>& indexes);

	void statisticalDistanceFilter(std::vector<double>& distances, rda::Range range, int neighbours_number, double threshold, std::vector<int>& indexes);

	void statisticalDistanceFilterDebug(std::vector<double>& distances, rda::Range range, int neighbours_number, double threshold, std::vector<int>& indexes, std::vector<double>& sum);
}

#endif