
#include <fstream>

#include "io.h"
#include "common.h"
#include <errno.h>

enum  SensorNum { LeftSensor = 1, FrontSensor, RightSensor};

void senorInFile(SensorNum num, int* size, int* offset){

	switch (num)
	{
	case LeftSensor:
		*offset = 4;
		*size = 2;
		break;
	case FrontSensor:
		*offset = 6;
		*size = 2;
		break;
	case RightSensor:
		*offset = 8;
		*size = 2;
		break;
	default:
		break;
	}
}

void client::readScene(std::string file, std::vector<double>& input, int& sensor){
	
	//setlocale(LC_ALL, "RUS");

	std::fstream f;
	f.open(file, std::ios::in);
	if(!f.is_open())
		throw client::RDAException("Failed to read file ");
	
	std::string buf;
	getline(f, buf);

	sensor = SensorNum::FrontSensor;
	int lineNumber = 0;
	int readFromLineNum = 0;
	int size = 4;		// количество считываемых элементов
	int offset = 1; // сдвиг относительно 0-ого
	//std::vector<double> points;

	std::vector<std::string> a;
	split(buf, " ", a);

	if (strcmp(a[2].c_str(), "0.") == 0){
		readFromLineNum = 1;
	}
	if (strcmp(a[2].c_str(), "1.") == 0){
		readFromLineNum = 16;
	}
	if (strcmp(a[2].c_str(), "2.") == 0){
		readFromLineNum = 27;		
		senorInFile(SensorNum::FrontSensor, &size, &offset);		
	}	
	std::vector<double> data;
	data.resize(size);

	bool isPoint = false;

	input.push_back(0); // size will be here

	while (!f.eof()){
		if (lineNumber >= readFromLineNum){
			std::vector<std::string> a;
			split(buf, " ", a);

			for (int i = 0; i < size - 1; i++){ // read x, y, angle
				if (a.back()[0] != 'n'){					
					data[i] = atof(a[offset + i].c_str());
					isPoint = true;
				}
				else {
					isPoint = false;
				}
			}
			data[size-1] = atof(a.back().c_str());

			if(isPoint){				
				
				input.push_back(data[0]); // x_rob
				input.push_back(data[1]); // y_rob
				input.push_back(data[2]); // angle
				input.push_back(data.back()); // distance
				input.push_back(1.0); // 
			}
		}

		if(lineNumber == 6){
			std::vector<std::string> a;
			split(buf, " ", a);
			if(strcmp(a.back().c_str(),"Rf_L:") == 0)
				sensor = SensorNum::LeftSensor;
			if(strcmp(a.back().c_str(),"Rf_R:") == 0)
				sensor = SensorNum::RightSensor;
		}

		getline(f, buf);
		lineNumber++;
	}

	input[0] = input.size();

	f.close();	
}

void client::readXYDScene(std::string file, std::vector<double>& input, int& sensor){
	
	//setlocale(LC_ALL, "RUS");

	std::fstream f;
	f.open(file, std::ios::in);
	if(!f.is_open())
		throw client::RDAException("Failed to read file ");
	
	std::string buf;
	getline(f, buf);

	sensor = SensorNum::FrontSensor;
	int lineNumber = 0;
	int readFromLineNum = 0;
	int size = 3;		// количество считываемых элементов
	int offset = 4; // сдвиг относительно 0-ого
	//std::vector<double> points;

	std::vector<std::string> a;
	split(buf, " ", a);

	if (strcmp(a[2].c_str(), "0.") == 0){
		readFromLineNum = 1;
	}
	if (strcmp(a[2].c_str(), "1.") == 0){
		readFromLineNum = 16;
	}
	if (strcmp(a[2].c_str(), "2.") == 0){
		readFromLineNum = 27;		
		senorInFile(SensorNum::FrontSensor, &size, &offset);		
	}	
	std::vector<double> data;
	data.resize(size);

	bool isPoint = false;

	input.push_back(0); // size will be here

	while (!f.eof()){
		if (lineNumber >= readFromLineNum){
			std::vector<std::string> a;
			split(buf, " ", a);

			for (int i = 0; i < size; i++){ // read x, y, d
				if (a.back()[0] != 'n'){					
					data[i] = atof(a[offset + i].c_str());
					isPoint = true;
				}
				else {
					isPoint = false;
				}
			}
			data[size-1] = atof(a.back().c_str());

			if(isPoint){				
				
				input.push_back(data[0]); // x
				input.push_back(data[1]); // y
				input.push_back(0.0); // 
				input.push_back(0.0); // 
				input.push_back(data.back()); // distance
			}
		}

		if(lineNumber == 6){
			std::vector<std::string> a;
			split(buf, " ", a);
			if(strcmp(a.back().c_str(),"Rf_L:") == 0)
				sensor = SensorNum::LeftSensor;
			if(strcmp(a.back().c_str(),"Rf_R:") == 0)
				sensor = SensorNum::RightSensor;
		}

		getline(f, buf);
		lineNumber++;
	}

	input[0] = input.size();

	f.close();	
}

void client::writeScene(std::string file, double** cloud, int size, int point_size){
	std::ofstream f;
	f.open(file);
	
	for(int i = 0; i < size; i++){
		
		for(int j = 1; j < cloud[i][0]-1; j+= point_size){			
			for(int k = 0; k < point_size; k++){
				f << cloud[i][j+k] << " ";
			}
			f << std::endl;
		}
		f << std::endl;
	}

	f.close();
	
}