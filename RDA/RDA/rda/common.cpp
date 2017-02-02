
#include <math.h>
#include <rda\common.h>
#include <rda\vector.h>
#include <rda\line.h>

void rda::split(std::string str, char* delimiter, std::vector<std::string>& parts)
{
	char* s = (char*)str.c_str();
	const char* d = delimiter;
 
	char *next_token1 = 0;
	char *token1 = 0;
	token1 = strtok_s(s, d, &next_token1);
	while ((token1 != 0)){
		if (token1 != 0)
		{
			parts.push_back(token1);
			token1 = strtok_s(0, d, &next_token1);
		}
	}	
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

double rda::distancePointToPoint(rda::Point& point_1, rda::Point& point_2)
{
	return sqrt( pow(point_2.x - point_1.x, 2) + pow(point_2.y - point_1.y, 2) );
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

double rda::distanceSegmentToSegment(rda::Line& line_1, rda::Line& line_2)
{	
	std::vector<double> dists;
	dists.push_back(rda::distancePointToSegment(line_1.start(), line_1.end(), line_2.start()));
	dists.push_back(rda::distancePointToSegment(line_1.start(), line_1.end(), line_2.end()));
	dists.push_back(rda::distancePointToSegment(line_2.start(), line_2.end(), line_1.start()));
	dists.push_back(rda::distancePointToSegment(line_2.start(), line_2.end(), line_1.end()));	
	return (*std::min_element(dists.begin(), dists.end()));
}

rda::BoundingBox rda::boundingBox(rda::CloudPtr cloud)
{
	rda::BoundingBox bb = { rda::Point(0,0,0), 0, 0 };

	if(cloud->size() > 0){
		rda::Point min(cloud->at(0));
		rda::Point max(cloud->at(0));

		for(auto i = 0; i < cloud->size(); i++){
			if(min.x > cloud->at(i).x)
				min.x = cloud->at(i).x;
			if(min.y > cloud->at(i).y)
				min.y = cloud->at(i).y;

			if(max.x < cloud->at(i).x)
				max.x = cloud->at(i).x;
			if(max.y < cloud->at(i).y)
				max.y = cloud->at(i).y;
		}
		
		bb.bottom = min;
		bb.height = max.y - min.y;
		bb.width = max.x - min.x;		
	}
	
	return bb;
}

rda::CloudPtr rda::boundingBoxVertices(rda::CloudPtr cloud)
{
	rda::CloudPtr bv(new rda::Cloud);
	rda::BoundingBox bb = rda::boundingBox(cloud);
	bv->push_back(bb.bottom);
	bv->push_back(rda::Point(bb.bottom.x, bb.bottom.y + bb.height, bb.bottom.z));
	bv->push_back(rda::Point(bb.bottom.x + bb.width , bb.bottom.y + bb.height, bb.bottom.z));
	bv->push_back(rda::Point(bb.bottom.x + bb.width, bb.bottom.y, bb.bottom.z));
	bv->push_back(bb.bottom);
	return bv;
}

void rda::rotateCloud(rda::CloudPtr cloud, double angle_rad, rda::CloudPtr rotated_cloud)
{	
	rda::rotateCloud(cloud, 0, cloud->size() - 1, angle_rad, rotated_cloud);
}

void rda::rotateCloud(rda::CloudPtr cloud, int start, int end, double angle_rad, rda::CloudPtr rotated_cloud)
{
	for(auto i = start; i <= end; i++){
		rda::Point& p = cloud->at(i);
		rotated_cloud->push_back(  rda::Point(p.x * cos(angle_rad) + p.y*sin(angle_rad), -p.x*sin(angle_rad) + p.y*cos(angle_rad), p.z) );
	}
}