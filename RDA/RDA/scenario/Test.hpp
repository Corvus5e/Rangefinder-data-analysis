
#ifndef RDA_TEST_H
#define RDA_TEST_H

#include <scenario\Scenario.h>

#include <rda\io\io.h>
#include <rda\io\console.h>
#include <rda\io\vizualization.h>
#include <rda\curve.h>
#include <rda\function\LeastSquares.h>


class Test : public Scanario{
public:

	void start(int argc, char* argv[])
	{
		rda::Console console;
		console.readArgs(argc, argv);	

		std::vector<double> distances;		
		std::vector<rda::Range> part_ranges;
		rda::CloudPtr cloud (new rda::Cloud);		
		std::vector<rda::Line> lines;
		
		for(auto i = 0; i < 20; i++){
			cloud->push_back(rda::Point(4.0f, 0.0f, 1.0f));
		}

		/*function::LeastSquares ls(1);

		ls.init(cloud, 0, cloud->size() - 1);
		ls.approximate();
		
		lines.push_back(rda::Line(rda::Point(cloud->front().x, ls.value(cloud->front().x), 1.0f),
								  rda::Point(cloud->back().x, ls.value(cloud->back().x), 1.0f)));*/
				
		lines.push_back(rda::lsLineApproximation(rda::CloudPart(cloud)));

		// Vizualization
		rda::Vizualizer::init(&argc, argv);	
		rda::Vizualizer v_1;			
	
		v_1.createWindow("Least", 720, 720, 2, 2);		

		v_1.addCloud(cloud, rda::CIRCLES, 1.0f, 1.0f, 1.0f, 0.3f);					
		for(auto i = 0; i < lines.size(); i++){
			v_1.addCloud(lines[i], rda::LINES, 1.0f, 0.0f, 0.0f, 1.0f);
		}


		rda::Vizualizer::start();	
	}
};

#endif