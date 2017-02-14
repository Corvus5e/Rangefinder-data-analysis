
#ifndef RDA_IO_H
#define RDA_IO_H

#include <iostream>
#include <string>
#include <map>
#include <vector>

#include <rda\common.h>


namespace rda {	
		
	pcl::PointCloud<pcl::PointXYZ>::Ptr readScene(std::string file, std::vector<double>& distances);

	pcl::PointCloud<pcl::PointXYZ>::Ptr readScene(std::string file, std::vector<double>& distances, std::vector<rda::Range>& part_ranges);

	void readScene(std::string file, std::vector<double>& distances, std::vector<rda::RPoint>& rob_points, int& sensor);

	void readScene(std::string file, std::vector<double>& distances, std::vector<rda::RPoint>& rob_points, std::vector<rda::Range>& parts, int& sensor);

}


#endif