
#ifndef RDA_CLIENT_IO_H
#define RDA_CLIENT_IO_H

#include <string>
#include <vector>


namespace client {

	void readScene(std::string file, std::vector<double>& data, int& sensor);	

	/*
	 * File format from 2nd line: Time, X_rob, Y_rob, Ang_rob, X_obj, Y_obj, Distance
	 * Reads all data ( with n/a - converted to zeros)
	*/
	void readXYDScene(std::string file, std::vector<double>& data);	

	void writeScene(std::string file, double** cloud, int size, int point_size);
}

#endif