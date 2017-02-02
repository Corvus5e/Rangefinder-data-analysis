 
#include <fstream>

#include <rda\io\io.h>
#include <rda\common.h>
#include <rda\rda_exception.h>


namespace {

	void senorInFile(rda::SensorNum num, int* size, int* offset)
	{
		switch (num)
		{
		case rda::LeftSensor:
			*offset = 4;
			*size = 2;
			break;
		case rda::FrontSensor:
			*offset = 6;
			*size = 2;
			break;
		case rda::RightSensor:
			*offset = 8;
			*size = 2;
			break;
		default:
			break;
		}
	}
}

pcl::PointCloud<pcl::PointXYZ>::Ptr rda::readScene(std::string file, std::vector<double>& distances)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZ> );
	//setlocale(LC_ALL, "RUS");

	std::fstream f;
	f.open(file, std::ios::in);
	if(!f.is_open())
		throw rda::RdaException("Failed to read file " + file);
	
	std::string buf;
	getline(f, buf);

	int lineNumber = 0;
	int readFromLineNum = 0;
	int size = 2; //+ 1;		// количество считываемых элементов
	int offset = 4; // сдвиг относительно 0-ого
	//std::vector<double> points;

	std::vector<std::string> a;
	split(buf, " ", a);

	if (strcmp(a[2].c_str(), "0.") == 0){
		readFromLineNum = 1;
	}
	if (strcmp(a[2].c_str(), "1.") == 0){
		readFromLineNum = 12;
	}
	if (strcmp(a[2].c_str(), "2.") == 0){
		readFromLineNum = 27;		
		senorInFile(FrontSensor, &size, &offset);		
	}	
	std::vector<double> data;
	data.resize(size);

	bool isPoint = false;

	while (!f.eof())
	{
		if (lineNumber++ >= readFromLineNum)
		{
			std::vector<std::string> a;
			split(buf, " ", a);

			for (int i = 0; i < size; i++){
				if (a[offset + i][0] != 'n'){					
					data[i] = atof(a[offset + i].c_str());		
					isPoint = true;
				}
				else {
					isPoint = false;
				}
			}

			if(isPoint){
				cloud->push_back(pcl::PointXYZ(data[0], data[1], 0.0));
				//distances.push_back(data[2]);
			}
		}

		getline(f, buf);
	}

	f.close();	
	
	return cloud;
}

void rda::readScene(std::string file, std::vector<double>& distances, std::vector<rda::RPoint>& rob_points, int& sensor){
	
	//setlocale(LC_ALL, "RUS");

	std::fstream f;
	f.open(file, std::ios::in);
	if(!f.is_open())
		throw rda::RdaException("Failed to read file " + file);
	
	std::string buf;
	getline(f, buf);

	sensor = rda::FrontSensor;
	int lineNumber = 0;
	int readFromLineNum = 0;
	int size = 4;		// количество считываемых элементов
	int offset = 1; // сдвиг относительно 0-ого
	//std::vector<double> points;

	std::vector<std::string> a;
	rda::split(buf, " ", a);

	if (strcmp(a[2].c_str(), "0.") == 0){
		readFromLineNum = 1;
	}
	if (strcmp(a[2].c_str(), "1.") == 0){
		readFromLineNum = 16;
	}
	if (strcmp(a[2].c_str(), "2.") == 0){
		readFromLineNum = 27;		
		senorInFile(FrontSensor, &size, &offset);		
	}	
	std::vector<double> data;
	data.resize(size);

	bool isPoint = false;

	while (!f.eof())
	{
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
				rob_points.push_back(rda::RPoint(data[0], data[1], data[2]));
				distances.push_back(data.back());
			}
		}

		if(lineNumber == 6){
			std::vector<std::string> a;
			rda::split(buf, " ", a);
			if(strcmp(a.back().c_str(),"Rf_L:") == 0)
				sensor = rda::LeftSensor;
			if(strcmp(a.back().c_str(),"Rf_R:") == 0)
				sensor = rda::RightSensor;
		}

		getline(f, buf);
		lineNumber++;
	}

	f.close();	
}

void rda::readScene(std::string file, std::vector<double>& distances, std::vector<rda::RPoint>& rob_points, std::vector<rda::Range>& parts, int& sensor)
{
	std::fstream f;
	f.open(file, std::ios::in);
	if(!f.is_open())
		throw rda::RdaException("Failed to read file " + file);

	sensor = FrontSensor;
	int line_number = 0;
	int read_from_line_num = 1;
	int size = 4;		// количество считываемых элементов
	int offset = 1; // сдвиг относительно 0-ого
	sensor = FrontSensor;

	std::string buf;
	getline(f, buf);
	std::vector<std::string> title_names;
	split(buf, " ", title_names);

	if (strcmp(title_names[2].c_str(), "0.") == 0){
		read_from_line_num = 1;
	}
	if (strcmp(title_names[2].c_str(), "1.") == 0){
		read_from_line_num = 16;
	}
	if (strcmp(title_names[2].c_str(), "2.") == 0){
		read_from_line_num = 27;		
		senorInFile(FrontSensor, &size, &offset);		
	}

	int no_data = 0;
	int start_index = 0;
	bool closed_part = true;
	while(!f.eof()){
		if(line_number >= read_from_line_num){
			std::vector<std::string> slices;
			split(buf, " ", slices);

			if(slices.back().at(0) != 'n'){
				if(closed_part){
					start_index = distances.size();
					closed_part = false;
				}
				RPoint r_point(atof(slices[1].c_str()), atof(slices[2].c_str()), atof(slices[3].c_str()));
				rob_points.push_back(r_point);
				distances.push_back(atof(slices.back().c_str()));
				no_data = 0;
			}
			else{
				if(++no_data == 3){
					if(!closed_part){
						parts.push_back(rda::Range(start_index, distances.size() - 1));
						closed_part = true;
					}
				}
			}
		}

		if(line_number == 6){
			std::vector<std::string> a;
			split(buf, " ", a);
			if(strcmp(a.back().c_str(),"Rf_L:") == 0)
				sensor = LeftSensor;
			if(strcmp(a.back().c_str(),"Rf_R:") == 0)
				sensor = RightSensor;
		}

		getline(f, buf);
		line_number++;
	}
	if(!closed_part)
		parts.push_back(rda::Range(start_index, distances.size() - 1));
}