
#ifndef RDA_CLIENT_IO_H
#define RDA_CLIENT_IO_H

#include <string>
#include <vector>


namespace client {

	void readScene(std::string file, std::vector<double>& data, int& sensor);	

	void readXYDScene(std::string file, std::vector<double>& data, int& sensor);	

	void writeScene(std::string file, double** cloud, int size, int point_size);
}

#endif