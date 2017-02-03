
#include <math.h>
#include <rda\common.h>
#include <rda\vector.h>

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

double rda::distancePointToPoint(rda::Point& point_1, rda::Point& point_2)
{
	return sqrt( pow(point_2.x - point_1.x, 2) + pow(point_2.y - point_1.y, 2) );
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