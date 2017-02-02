
#ifndef RDA_IO_H
#define RDA_IO_H

#include <iostream>
#include <string>
#include <map>
#include <vector>

#include <pcl\point_types.h>
#include <pcl\point_cloud.h>

#include <rda\common.h>


namespace rda {	

	enum  SensorNum { LeftSensor = 1, FrontSensor = 2, RightSensor = 3};
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr readScene(std::string file, std::vector<double>& distances);

	void readScene(std::string file, std::vector<double>& distances, std::vector<rda::RPoint>& rob_points, int& sensor);

	void readScene(std::string file, std::vector<double>& distances, std::vector<rda::RPoint>& rob_points, std::vector<rda::Range>& parts, int& sensor);

}


#endif