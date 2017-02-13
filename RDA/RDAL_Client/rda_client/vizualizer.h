
#ifndef RDA_CLIENT_VIZUALIZER_H
#define RDA_CLIENT_VIZUALIZER_H

#include <vector>

#include "GL\glut.h"
#include "point_cloud.h"

namespace client {

	typedef struct {
				float r;
				float g;
				float b;
				float alpha;
			} Color;

	enum CLIENT_DRAW_MODE {
		POINTS = GL_POINTS,
		LINES = GL_LINES,
		LINE_STRIP = GL_LINE_STRIP
	} ;

	class Vizualizer {

	private :

		std::vector<client::PointCloud> clouds_;
		std::vector<int> modes_;
		std::vector<Color> colors_;
		std::vector<float> pointSizes_;

		int width_;
		int height_;

		double eyeX;
		double eyeY;
		double eyeZ;
		int lastX;
		int lastY;
		static const int maxScale = 500;

		int index;

	public:

		Vizualizer();

		static void init(int* argc, char* argv[]);

		static void start();

		void createWindow(const char* title, int windth, int height, int x, int y);

		void addCloud(client::PointCloud cloud, int mode, float r, float g, float b, float alpha, float pointSize = 2.0f);

		void addClouds(std::vector<client::PointCloud>& clouds, int mode, float alpha, float pointSize = 2.0f);

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