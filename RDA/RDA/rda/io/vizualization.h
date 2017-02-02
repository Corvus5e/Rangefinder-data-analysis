
#ifndef VIZUALIZATION_H
#define VIZUALIZATION_H

#include <vector>
#include <map>

#include <rda\io\GL\glut.h>
#include <pcl\point_types.h>
#include <pcl\point_cloud.h>

#include <rda\common.h>
#include <rda\cloud_part.h>
#include <rda\approximied_cloud_part.h>

namespace rda {

	typedef struct {
				float r;
				float g;
				float b;
				float alpha;
			} Color;

	enum RDA_DRAW_MODE {
		POINTS = GL_POINTS,
		LINES = GL_LINES,
		LINE_STRIP = GL_LINE_STRIP,
		CIRCLES = 76
	} ;

	class Vizualizer {

	private:		

		int width_;
		int height_;

		double eyeX;
		double eyeY;
		double eyeZ;
		int lastX;
		int lastY;
		static const int maxScale = 10000;
		
		std::vector<CloudPtr> clouds_;
		std::vector<int> modes_;
		std::vector<Color> colors_;
		std::vector<float> pointSizes_;

		int index;

		static std::map<int, rda::Vizualizer*> objs; // id of gl windows -> pointer to Vizualizer object

	public:

		Vizualizer();

		static void init(int* argc, char* argv[]);

		static void start();

		void createWindow(const char* title, int windth, int height, int pos_x, int pos_y);

		void addCloud(CloudPtr cloud, int mode, float r, float g, float b, float alpha, float pointSize = 2.0f);

		void addCloud(rda::Line line, int mode, float r, float g, float b, float alpha, float pointSize = 2.0f);

		void addCloud(rda::CloudPart& part, int mode, float r, float g, float b, float alpha, float pointSize = 2.0f);

		void addCloud(rda::ApproximiedCloudPart& part, int mode, float r, float g, float b, float alpha, float pointSize = 2.0f);

		void addClouds(std::vector<rda::Line>& lines, int mode, float alpha, float pointSize = 2.0f);

		void addClouds(std::vector<CloudPtr>& clouds, int mode, float alpha, float pointSize = 2.0f);

		void addClouds(std::vector<rda::CloudPart>& clouds, int mode, float alpha, float pointSize = 2.0f);

		void addClouds(std::vector<rda::ApproximiedCloudPart>& clouds, int mode, float alpha, float pointSize = 2.0f);

		~Vizualizer();

	private:

		friend void global_draw();

		friend void global_mouse(int button, int state, int x, int y);

		friend void global_motion(int x, int y);

		friend void global_reshape(int width, int heigth);

		friend void global_key(unsigned char key, int x, int y);

		void draw();

		void mouseClick(int button, int state, int x, int y);

		void mouseMotion(int x, int y);

		void reshape(int w, int h);

		void keyPress(unsigned char key, int x, int y);

	};

}

#endif