
/*
 * Functions only for sorted point clouds 
*/

#ifndef RDA_RANGEFINDER_H_
#define RDA_RANGEFINDER_H_

#include <vector>

#include <rda\common.h>
#include <rda\cloud_part.h>
#include <rda\approximied_cloud_part.h>

namespace rda {	
	
	void computePointCloud(std::vector<rda::RPoint> rob_point, std::vector<double> distances, rda::CloudPtr cloud, int sensor);

	void computePointCloud(std::vector<rda::RPoint> rob_point, std::vector<double> distances, std::vector<int>& indexes, rda::CloudPtr cloud, int sensor);

	rda::Point computePoint(RPoint& rob_point, double distance, int sensor_id);	

	void naive_beakpoint_detector(std::vector<double>& distances, double max_diff, int min_points, std::vector<rda::Range>& indexes);

	void naive_beakpoint_detector(std::vector<double>& distances, std::vector<int>& v_indexes, double max_diff, int min_points, std::vector<std::vector<int>>& indexes);	

	void adaptiveNaiveDetector(std::vector<double>& distances, std::vector<std::pair<double, double>>& errors, int min_points, std::vector<rda::Range>& indexes);
}

#endif
