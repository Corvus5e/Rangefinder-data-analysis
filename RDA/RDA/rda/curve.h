
#ifndef RDA_CURVE_H
#define RDA_CURVE_H

#include <pcl\point_types.h>
#include <pcl\point_cloud.h>

#include <rda\common.h>
#include <rda\cloud_part.h>
#include <rda\approximied_cloud_part.h>
#include <rda\vector.h>
#include <rda\line.h>

namespace rda {

	double distanceSegmentToSegment(rda::Line& line_1, rda::Line& line_2);

	double distancePointToLine(pcl::PointXYZ& line_start, pcl::PointXYZ& line_end, pcl::PointXYZ& point);

	double distancePointToSegment(rda::Point& segment_start, rda::Point& segment_end, rda::Point& point);		
	
	//Ramer Douglas Peucker algorithm
	void rdpMinimization(rda::CloudPart cloud_part, int start, int end, double threshold, std::vector<rda::CloudPart>& lines);

	// significance = length / (max distance from endline)
	double simpleSignificanceEstimator(rda::CloudPart cloud, double& error, int& index);

	// significance = length / (standart deviation of distances from endline)
	double stDevSignificanceEstimator(rda::CloudPart cloud, double& error, int& index);	

	double stDevDirSignificanceEstimator(rda::CloudPart cloud, double& error, int& index);

	// significance = length / (mean of distances from endline) //distance can be negative
	double meanSignificanceEstimator(rda::CloudPart cloud, double& error, int& index);	

	double adaptiveRDP(rda::CloudPart cloud, double min_error, int min_size, std::vector<rda::CloudPart>& line_parts, double (*significanceEstimator)(rda::CloudPart, double&, int&) = simpleSignificanceEstimator);

	void incrementalLineFit(rda::CloudPart cloud, double threashold, int points_increment, int stable_angles, std::vector<rda::Line>& lines);

	void simpleMovingAvarage(rda::CloudPtr cloud, int window_size, rda::CloudPtr sma_cloud);

	// value of overlaping of projections of line_1 anf line_2 undo middle_line
	rda::Line middleLine(rda::ApproximiedCloudPart acp_1, rda::ApproximiedCloudPart acp_2, double* overlaping);

	rda::Point projectionPointToLine(rda::Line line, rda::Point& point);

	rda::Point projectionPointToLine(double line_angle, Point& line_mid_point, Point& point);

	//projects line_1 to line_2 
	rda::Line projectionLineToLine(rda::Line line_1, rda::Line line_2);

	rda::Line maxDiagonal(rda::Line& line_1, rda::Line& line_2);

	//Finds distances from i to i+1 point 
	void distances(rda::CloudPtr cloud, std::vector<double>& dists);

	void monotonePartitioning(rda::CloudPtr cloud, double max_dist, int min_part_size, std::vector<rda::CloudPart>& parts);

	void naiveBreakpointDetector(rda::CloudPtr cloud, std::vector<int>& v_indexes, double max_diff, int min_points, std::vector<std::vector<int>>& indexes);

	void naiveBreakpointDetector(rda::CloudPart cloud, double max_diff, int min_points, std::vector<std::vector<int>>& indexes);

	void lineSegmentation(std::vector<rda::CloudPart>& parts, double threshold, std::vector<rda::CloudPart>& line_parts);	

	void adaptiveLineSegmentation(std::vector<rda::CloudPart>& parts, int min_part_size, double min_error, std::vector<rda::CloudPart>& line_parts);

	//Least Squares Line Approximation
	void lsLineApproximation(std::vector<rda::CloudPart>& parts, std::vector<rda::ApproximiedCloudPart>& line_approx);

	void lsRDPApproximation(rda::CloudPart cloud_part, int order, double step, double threashold, std::vector<rda::CloudPart>& line_approx);

	rda::Line lsLineApproximation(rda::CloudPart& part);

	void segmentsMerging(std::vector<rda::ApproximiedCloudPart> segments, double dist_threashold, double angle_threashold, std::vector<rda::ApproximiedCloudPart>& merged_segments);

	bool areSimilar(Line& line_1, Line& line_2, double dist_threashold, double angle_threashold);	
}

#endif