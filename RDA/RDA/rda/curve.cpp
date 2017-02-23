
#include <cmath>
#include <vector>
#include <list>

#include <rda\curve.h>
#include <rda\common.h>
#include <rda\function\LeastSquares.h>

using namespace std;
using namespace rda;
using namespace function;

namespace {

	//TODO add maxDistanceFromLsLine from Eric\curve.cpp

	double maxDistanceFromLine(rda::CloudPtr cloud, int start_index, int end_index, int& index){		
		
		index = start_index;
		double max_dist = 0.0;		
	
		if(end_index - start_index > 1){			
			double curr_dist = 0;	
			for(int i = start_index + 1; i < end_index; i++){
				curr_dist = rda::distancePointToLine(cloud->at(start_index), cloud->at(end_index), cloud->at(i));	
				if(curr_dist > max_dist){
					max_dist = curr_dist;
					index = i;
				}
			}
		}
		return max_dist;
	}
	
	double maxDistanceFromLine(rda::CloudPtr cloud, int start_index, int end_index, double& avarage, double& deviation, int& index){
	
		avarage = 0.0;
		deviation = 0.0;
		index = start_index;

		double max_dist = 0.0;		
		vector<double> dists;		
	
		if(end_index - start_index > 1){									
			double curr_dist = 0;
			for(int i = start_index + 1; i < end_index; i++){
				curr_dist = rda::distancePointToLine(cloud->at(start_index), cloud->at(end_index), cloud->at(i));				
				if(curr_dist > max_dist){
					max_dist = curr_dist;
					index = i;
				}
				dists.push_back(curr_dist);
			}
			deviation = rda::standartDeviation(dists.begin(), dists.end(), avarage);
		}
		return max_dist;
	}

	bool line_cmp(Line line_1, Line line_2){
		return line_1.length() < line_2.length();
	}

	rda::Line extendLine(rda::ApproximiedCloudPart p1, rda::ApproximiedCloudPart p2){
		rda::Line pl = rda::projectionLineToLine(p2.approx_line(), p1.approx_line());
		return rda::maxDiagonal(p1.approx_line(), pl);	
	}
	
	rda::Line mergeLines(rda::ApproximiedCloudPart l1, rda::ApproximiedCloudPart l2){
		rda::CloudPtr cloud ( new rda::Cloud);
		double overlapping = 0.0;
		return rda::middleLine(l1, l2, &overlapping);	
	}

	template<class InputIterator>
	bool stable(InputIterator begin, InputIterator end, double eps)
	{
		auto s = *(begin + 1) - *begin;
		while(begin != (end - 1)){
			auto diff = *(begin + 1) - *begin;
			if((s / diff) <= 0)
				return false;
			++begin;
		}
		return true;
	}
}

double rda::distanceSegmentToSegment(rda::Line& line_1, rda::Line& line_2)
{	
	std::vector<double> dists;
	dists.push_back(rda::distancePointToSegment(line_1.start(), line_1.end(), line_2.start()));
	dists.push_back(rda::distancePointToSegment(line_1.start(), line_1.end(), line_2.end()));
	dists.push_back(rda::distancePointToSegment(line_2.start(), line_2.end(), line_1.start()));
	dists.push_back(rda::distancePointToSegment(line_2.start(), line_2.end(), line_1.end()));	
	return (*std::min_element(dists.begin(), dists.end()));
}

double rda::distancePointToLine(pcl::PointXYZ& line_start, pcl::PointXYZ& line_end, pcl::PointXYZ& point)
{
	/*
	 * Theory in
	 * http://www.cleverstudents.ru/line_and_plane/distance_from_point_to_line.html
	 * 
	 */

	double A = (line_end.y - line_start.y);
	double B = (line_start.x - line_end.x);
	double C =  - line_end.y * line_start.x + (line_start.y * line_end.x);
	double k = - 1.0 / (sqrt((A*A) + (B*B)));

	return abs(k*(A*point.x + B*point.y + C));	
}

void rda::rdpMinimization(rda::CloudPart cloud_part, int start, int end, double threshold, std::vector<rda::CloudPart>& lines){

	if(cloud_part.size() <= 2){		

		lines.push_back(rda::CloudPart(cloud_part.cloud(), cloud_part.range()));
	}
	else {

		int index_0 = 0;

		double dist_0 = maxDistanceFromLine(cloud_part.cloud(), start, end, index_0);		

		if( (dist_0 > threshold) ){ 

			std::vector<rda::CloudPart> line_1;
			std::vector<rda::CloudPart> line_2;
			
			rdpMinimization(cloud_part, start, index_0, threshold, line_1);
			rdpMinimization(cloud_part, index_0, end, threshold, line_2);

			for(int i=0; i < line_1.size(); i++)
				lines.push_back(line_1.at(i));
			for(int i=0; i < line_2.size(); i++)
				lines.push_back(line_2.at(i));

		}
		else {			
			lines.push_back(rda::CloudPart(cloud_part.cloud(), rda::Range(start, end)));			
		}
	}
}

void rda::incrementalLineFit(rda::CloudPart cloud, double threashold, int points_increment, int stable_angles, std::vector<rda::Line>& lines)
{
	int start_index = cloud.range().start;
	int end_index = start_index + points_increment;
	std::vector<double> angles;

	Line start_line = rda::CloudPart(cloud.cloud(), Range(start_index, end_index)).line();//lsLineApproximation(rda::CloudPart(cloud.cloud(), Range(start_index, end_index)));
	end_index += points_increment;

	while(end_index + points_increment < cloud.range().end){
		Line line = lsLineApproximation(rda::CloudPart(cloud.cloud(), Range(start_index, end_index)));//rda::CloudPart(cloud.cloud(), Range(start_index, end_index)).line();		
		angles.push_back(Vector::angle(start_line.directionVector(), line.directionVector()));

		if(angles.size() >= stable_angles){
			if(*std::max_element(angles.begin(), angles.end()) > threashold && stable(angles.end() - stable_angles, angles.end(), threashold)){
				lines.push_back(lsLineApproximation(rda::CloudPart(cloud.cloud(), Range(start_index, end_index - points_increment*stable_angles))));
				//lines.push_back(rda::CloudPart(cloud.cloud(), Range(start_index, end_index - points_increment*stable_angles)).line());
				start_index = end_index - points_increment * stable_angles;
				end_index = start_index + points_increment;
				start_line = lsLineApproximation(rda::CloudPart(cloud.cloud(), Range(start_index, end_index)));//rda::CloudPart(cloud.cloud(), Range(start_index, end_index)).line();				
				angles.clear();
			}
		}
		end_index += points_increment;				
	}
	lines.push_back(lsLineApproximation(rda::CloudPart(cloud.cloud(), Range(start_index, cloud.range().end))));
	//lines.push_back((rda::CloudPart(cloud.cloud(), Range(start_index, cloud.range().end)).line()));
}

double rda::distancePointToSegment(rda::Point& segment_start, rda::Point& segment_end, rda::Point& point)
{	
	/* Theory http://algolist.manual.ru/maths/geom/distance/pointline.php */
	
	rda::Vector v(segment_start, segment_end);
	rda::Vector w(segment_start, point);

	double c1 = v * w;
	double c2 = v * v;

	if( c1 <= 0 )
		return rda::distancePointToPoint(segment_start, point);	

	if(c2 <= c1)
		return rda::distancePointToPoint(segment_end, point);

	double t = c1 / c2;

	/*  parametric line 
	 *	x(t) = x1(1-t) + x*t
	 *  y(t) = y1(1-t) + y*t,
	 *  0 <= t <= 1
	*/
	
	// pb is projection of p on vector v
	rda::Point pb(segment_start.x * (1.0 - t) + segment_end.x * t, segment_start.y * (1.0 - t) + segment_end.y * t, 0);

	return rda::distancePointToPoint(point, pb);
}

void rda::simpleMovingAvarage(rda::CloudPtr cloud, int window_size, rda::CloudPtr sma_cloud){	

	for(int i =  window_size - 1; i < cloud->size(); i++){

		double sma_t = 0; // value of y in x_t

		for(int j = 0; j < window_size; j++){
			sma_t += cloud->at(i - j).y;
		}

		sma_t /= window_size;

		sma_cloud->push_back(rda::Point(cloud->at(i).x, sma_t, cloud->at(i).z));

	}
}

rda::Point rda::projectionPointToLine(rda::Line line, rda::Point& point){
	rda::Vector norm = line.normVector();
	double k = norm.y/norm.x; // k of line perpendicular to mid_line and passing through point "point"
	double b = (norm.x*point.y - norm.y * point.x)/ norm.x;
	Line cl = Line(Point(0, 0*k + b, 0),Point(1000, 1000*k + b, 0));
	Point p;
	Line::intersection(line, cl, &p);
	return p;
}

rda::Line rda::projectionLineToLine(rda::Line line_1, rda::Line line_2){
	return rda::Line(rda::projectionPointToLine(line_2, line_1.start()), rda::projectionPointToLine(line_2, line_1.end())) ;
}

rda::Point rda::projectionPointToLine(double line_angle, Point& line_mid_point, Point& point){
	
	Line mid_line = Line(line_mid_point, Point(0, std::tan(line_angle * 3.1415 / 180.0)*(0 - line_mid_point.x) + line_mid_point.y, line_mid_point.z));
	rda::Vector norm = mid_line.normVector();
	double k = norm.y/norm.x; // k of line perpendicular to mid_line and passing through point "point"
	double b = (norm.x*point.y - norm.y * point.x)/ norm.x;
	Line cl = Line(Point(0, 0*k + b, 0),Point(1000, 1000*k + b, 0));
	Point p;
	Line::intersection(mid_line, cl, &p);
	return p;
}

rda::Line rda::maxDiagonal(rda::Line& line_1, rda::Line& line_2){
	// looking for two points with separated by the largest distance
	std::vector<Line> lines;
	lines.push_back(rda::Line(line_1.start(), line_2.start()));
	lines.push_back(rda::Line(line_1.start(), line_2.end()));
	lines.push_back(rda::Line(line_1.end(), line_2.start()));
	lines.push_back(rda::Line(line_1.end(), line_2.end()));
	lines.push_back(line_1);
	lines.push_back(line_2);

	return (*std::max_element(lines.begin(), lines.end(), line_cmp));
}

rda::Line rda::middleLine(rda::ApproximiedCloudPart acp_1, rda::ApproximiedCloudPart acp_2, double* overlaping){

	Line line_1 = acp_1.approx_line();
	Line line_2 = acp_2.approx_line();

	Line ox(Point(0, 0, 0), Point(1, 0, 0)); // Ox axis

	double angle_1 = Vector::angle(ox.directionVector(), line_1.directionVector());
	if( line_1.k() < 0) 
		angle_1 = 180 - angle_1;		

	double angle_2 = Vector::angle(ox.directionVector(), line_2.directionVector());
	if( line_2.k() < 0) 
		angle_2 = 180 - angle_2;		

	if(angle_1 < angle_2)
		std::swap(angle_1, angle_2);

	if((angle_1 - angle_2) > 90){
		angle_1 = angle_1 - 180;
	}

	double len_1 = line_1.length();
	double len_2 = line_2.length();

	double mid_angle = ( acp_1.size() * angle_1 + acp_2.size() * angle_2 ) / (acp_1.size() + acp_2.size());
	/*Point mid_mid_point((len_1 * line_1.middle_point().x + len_2 * line_2.middle_point().x) / ( len_1 + len_2), 
						(len_1 * line_1.middle_point().y + len_2 * line_2.middle_point().y) / ( len_1 + len_2), 
						(len_1 * line_1.middle_point().z + len_2 * line_2.middle_point().z) / ( len_1 + len_2));*/

	Point mid_mid_point((acp_1.size() * line_1.middlePoint().x + acp_2.size() * line_2.middlePoint().x) / ( acp_1.size() + acp_2.size()), 
						(acp_1.size() * line_1.middlePoint().y + acp_2.size() * line_2.middlePoint().y) / ( acp_1.size() + acp_2.size()), 
						(acp_1.size() * line_1.middlePoint().z + acp_2.size() * line_2.middlePoint().z) / ( acp_1.size() + acp_2.size()));
	
	rda::Line max_line = rda::maxDiagonal(line_1, line_2);
	
	rda::Line mid_line =  Line(projectionPointToLine(mid_angle, mid_mid_point, max_line.start()), projectionPointToLine(mid_angle, mid_mid_point, max_line.end()));
	rda::Line proj_line_1  = rda::projectionLineToLine(line_1, mid_line);
	rda::Line proj_line_2  = rda::projectionLineToLine(line_2, mid_line);

	*overlaping =  -(mid_line.length() - proj_line_1.length() - proj_line_2.length());
	return mid_line;
}

double rda::simpleSignificanceEstimator(rda::CloudPart cloud, double& error, int& index)
{
	error = std::max(maxDistanceFromLine(cloud.cloud(), cloud.range().start, cloud.range().end, index), 3.0);
	return cloud.line().length() / error;
}

double rda::stDevSignificanceEstimator(rda::CloudPart cloud, double& error, int& index)
{	
	double avarage = 0;
	double st_d = 0;
	maxDistanceFromLine(cloud.cloud(), cloud.range().start, cloud.range().end, avarage, st_d, index); 

	error = st_d;
	return cloud.line().length() / st_d;
}

double rda::adaptiveRDP(rda::CloudPart cloud, double min_error, int min_size, std::vector<rda::CloudPart>& line_parts, double (*significanceEstimator)(rda::CloudPart, double&, int&))
{
	int mid_index;
	double error;
	double significance = significanceEstimator(cloud, error, mid_index);
	
	if(cloud.range().size() >= min_size){
		if(error > min_error){
			std::vector<rda::CloudPart> left_lines;
			std::vector<rda::CloudPart> right_lines;
			double left_significance = adaptiveRDP(rda::CloudPart(cloud.cloud(),rda::Range(cloud.range().start, mid_index)), min_error, min_size, left_lines, significanceEstimator);
			double right_significance = adaptiveRDP(rda::CloudPart(cloud.cloud(), rda::Range(mid_index, cloud.range().end)), min_error, min_size, right_lines, significanceEstimator);

			if(significance < left_significance || significance < right_significance){
				line_parts.insert(line_parts.end(), left_lines.begin(), left_lines.end());
				line_parts.insert(line_parts.end(), right_lines.begin(), right_lines.end());
				return std::max(left_significance, right_significance);
			}			
		}
	}
	else{		
		significance = -1;
	}

	line_parts.push_back(cloud);

	return significance;
}

void rda::distances(rda::CloudPtr cloud, std::vector<double>& dists)
{
	for(int i = 0; i < cloud->size()-1; i++){
		dists.push_back(rda::distancePointToPoint(cloud->at(i), cloud->at(i+1)));		
	}
}

void rda::monotonePartitioning(rda::CloudPtr cloud, double max_dist, int min_part_size, std::vector<rda::CloudPart>& parts)
{	
	std::vector<double> dists;
	distances(cloud, dists);	

	int start = 0;	

	for(int i = 0; i < dists.size(); i++){

		if( (max_dist < dists[i]) || i == (dists.size() - 1) ){

			if(i - start >= min_part_size){
				if(i == (dists.size() - 1))
					parts.push_back(rda::CloudPart(cloud, Range(start, i+1)));	
				else
					parts.push_back(rda::CloudPart(cloud, Range(start, i)));					
			}
			start = i + 1;
		}
	}
}

void rda::naiveBreakpointDetector(rda::CloudPtr cloud, std::vector<int>& v_indexes, double max_diff, int min_points, std::vector<std::vector<int>>& indexes)
{
	std::vector<int> tmp;
	
	for(auto i = 0; i < v_indexes.size() - 1; i++){

		tmp.push_back(v_indexes[i]);

		if( std::abs( rda::distancePointToPoint(cloud->at(v_indexes[i]), cloud->at(v_indexes[i+1])) ) > max_diff ){
			if( tmp.size() >= min_points)
				indexes.push_back(tmp);	
			tmp.clear();
		}		
	}

	tmp.push_back(v_indexes.back());
	if(tmp.size() >= min_points)
		indexes.push_back(tmp);	
}

void rda::lineSegmentation(std::vector<rda::CloudPart>& parts, double threshold, std::vector<rda::CloudPart>& line_parts)
{	
	for(int i=0; i < parts.size(); i++){		
		rda::rdpMinimization(parts[i], parts[i].range().start, parts[i].range().end, threshold, line_parts);
	}
}

void rda::adaptiveLineSegmentation(std::vector<rda::CloudPart>& parts, int min_part_size, double min_error,std::vector<rda::CloudPart>& line_parts)
{
	for(int i=0; i < parts.size(); i++){		
		rda::adaptiveRDP(parts[i], min_error, min_part_size, line_parts);
	}
}

void rda::lsLineApproximation(std::vector<rda::CloudPart>& parts, std::vector<rda::ApproximiedCloudPart>& line_approx)
{
	for(int i = 0; i < parts.size(); i++){

		rda::Line corr_line(parts[i].first_point(), parts[i].last_point());
		rda::Vector oX(1, 0);
		
		double angle = rda::Vector::angle(corr_line.directionVector(), oX) * rda::pi / 180.0;

		if(corr_line.k() > 0)
			angle *= -1;

		rda::CloudPtr rotated_cloud (new rda::Cloud);
		rda::rotateCloud(parts[i].cloud(), parts[i].range().start, parts[i].range().end, -angle , rotated_cloud);		

		LeastSquares ls(1);
		ls.init(rotated_cloud, 0, rotated_cloud->points.size() - 1);
		ls.approximate();

		rda::CloudPtr appr_line_cloud (new rda::Cloud);
		appr_line_cloud->push_back(rda::Point(rotated_cloud->at(0).x, ls.value(rotated_cloud->at(0).x), rotated_cloud->at(0).z)); 
		appr_line_cloud->push_back(rda::Point(rotated_cloud->back().x, 
											  ls.value(rotated_cloud->back().x), 
											  rotated_cloud->back().z));

		rda::CloudPtr unrotated_cloud (new rda::Cloud);
		rda::rotateCloud(appr_line_cloud,  angle, unrotated_cloud);

		rda::Line line(unrotated_cloud->front(), unrotated_cloud->back());
		//rda::Line line(appr_line_cloud->front(), appr_line_cloud->back());

		line_approx.push_back(rda::ApproximiedCloudPart(parts[i], line));
	}
}

rda::Line rda::lsLineApproximation(rda::CloudPart& part){

	rda::Line corr_line(part.first_point(), part.last_point());
	rda::Vector oX(1, 0);
	
	double angle = rda::Vector::angle(corr_line.directionVector(), oX) * pi / 180.0;
	
	if(corr_line.k() > 0)
		angle *= -1;
	
	rda::CloudPtr rotated_cloud (new rda::Cloud);
	rda::rotateCloud(part.cloud(), part.range().start, part.range().end, -angle , rotated_cloud);		
	
	LeastSquares ls(1);
	ls.init(rotated_cloud, 0, rotated_cloud->points.size() - 1);
	ls.approximate();
	
	rda::CloudPtr appr_line_cloud (new rda::Cloud);
	appr_line_cloud->push_back(rda::Point(rotated_cloud->at(0).x, ls.value(rotated_cloud->at(0).x), rotated_cloud->at(0).z)); 
	appr_line_cloud->push_back(rda::Point(rotated_cloud->back().x, 
										  ls.value(rotated_cloud->back().x), 
										  rotated_cloud->back().z));
	
	rda::CloudPtr unrotated_cloud (new rda::Cloud);
	rda::rotateCloud(appr_line_cloud,  angle, unrotated_cloud);
	
	rda::Line line(unrotated_cloud->front(), unrotated_cloud->back());	
	
	return line;
}

// copy of vector
void rda::segmentsMerging(std::vector<rda::ApproximiedCloudPart> segments, double dist_threashold, double angle_threashold, std::vector<rda::ApproximiedCloudPart>& merged_segments){

	if(segments.size() < 1) 
		return;

	for(std::size_t i = 0; i < segments.size(); i++ ){

		for(std::size_t j = 0; j < segments.size(); j++){

			if(i != j){
				if(rda::areSimilar(segments[i].approx_line(), segments[j].approx_line(), dist_threashold, angle_threashold)){
					double d = segments[j].approx_line().length() / segments[i].approx_line().length();					
					double m = segments[j].size() / (double)segments[i].size();
					if(m <= 1.0){ // j length is smaller than i length
						double overlapping = 0.0;
						Line mid_line = rda::middleLine(segments[i], segments[j], &overlapping);
						if( overlapping/mid_line.length() > 0.3) {							
							if(m > 0.25){
								segments[i].set_approx_line(mid_line); //merge
							}
							else{
								segments[i].set_approx_line(extendLine(segments[i], segments[j])); //merge
							}
						}
						else{
							segments[i].set_approx_line(extendLine(segments[i], segments[j])); //merge						
						}

						segments.erase(segments.begin() + j );
						if(j < i)
							i--;
						j--;
					}
				}
			}
		}
	}

	for(std::size_t i = 0; i < segments.size(); i++)
		merged_segments.push_back(segments[i]);
}

bool rda::areSimilar(Line& line_1, Line& line_2, double dist_threashold, double angle_threashold){
	if( (rda::distanceSegmentToSegment(line_1, line_2) <= dist_threashold ) &&
		( rda::Vector::angle(line_1.directionVector(), line_2.directionVector()) <= angle_threashold) )
		return true;	
	return false;
}
