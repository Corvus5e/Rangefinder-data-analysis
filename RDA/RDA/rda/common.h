
#ifndef RDA_COMMON_H
#define RDA_COMMON_H

#include <vector>
#include <string>

#include <pcl\point_cloud.h>
#include <pcl\point_types.h>


namespace rda {

	const double pi  = 3.1415926535897;

	enum  SensorNum { LeftSensor = 1, FrontSensor = 2, RightSensor = 3};

	typedef pcl::PointXYZ Point;

	typedef pcl::PointCloud<pcl::PointXYZ> Cloud;

	typedef pcl::PointCloud<pcl::PointXYZ>::Ptr CloudPtr;

	struct RPoint {
		double x_robot;
		double y_robot;
		double angle_robot;

		RPoint(double x_rob, double y_rob, double angle_rob) : x_robot(x_rob), y_robot(y_rob), angle_robot(angle_rob) { }
	};

	struct Range {

		int start;
		int end;
		
		Range(int start = 0, int end = 0) : start(start), end(end) { }

		int size() { return std::abs(end - start + 1); }
	};	

	typedef struct { Point bottom; double height; double width; } BoundingBox;	

	void split(std::string str, char* delimiter, std::vector<std::string>& parts);
	
	double distancePointToPoint(rda::Point& point_1, rda::Point& point_2);	

	template<typename T, class InputIterator>
	T meanValue(InputIterator begin, InputIterator end)
	{
		T mean = T(0);
		auto size = end - begin;

		while(begin != end){
			mean += *begin;
			++begin;
		}
		return mean / size;
	}

	template<typename T, class InputIterator>
	T medianValue(InputIterator begin, InputIterator end)
	{
		T mean = T(0);
		auto size = end - begin;
		std::vector<T> tmp(size);		
		std::copy(begin, end, tmp.begin());
		std::sort(tmp.begin(), tmp.end());

		return tmp[size / 2];
	}

	template<typename T, class InputIterator>
	T standartDeviation(InputIterator begin, InputIterator end, T& mean)
	{
		mean = meanValue<T>(begin, end);
		T sum = 0;
		auto size = end - begin;

		while(begin != end){
			sum += pow(*begin - mean, 2);
			++begin;
		}

		return sqrt(sum / size);
	}

	BoundingBox boundingBox(rda::CloudPtr cloud);

	//returns clouPtr with vertexes of bounding box
	rda::CloudPtr boundingBoxVertices(rda::CloudPtr cloud);

	void rotateCloud(rda::CloudPtr cloud, double angle_rad, rda::CloudPtr ratated_cloud);

	void rotateCloud(rda::CloudPtr cloud, int start, int end, double angle_rad, rda::CloudPtr ratated_cloud);

	// matrix is size * size ; vecot is size
	template <size_t N>
	std::vector<double> mulMatrixOnVector(double (&matrix)[N][N], std::vector<double>& vector){
		std::vector<double> result;
		for(auto i = 0; i < N; i++){
			double el = 0.0;
			for(auto j = 0; j < N; j++){
				el += matrix[i][j] * vector[j]; 
			}
			result.push_back(el);
		}
		return result;
	}
}

#endif