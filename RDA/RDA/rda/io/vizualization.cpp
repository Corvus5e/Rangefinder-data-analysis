
#include <iostream>
#include <vector>
#include <map>

#include <rda\io\vizualization.h>
#include <rda\io\GL\glut.h>

std::map<int, rda::Vizualizer*> rda::Vizualizer::objs; // id of gl windows -> pointer to Vizualizer object

void rda::global_draw()
{		
	if(Vizualizer::objs.find(glutGetWindow()) != Vizualizer::objs.end()){
		Vizualizer::objs[glutGetWindow()]->draw();
	}
}

void rda::global_mouse(int button, int state, int x, int y)
{
	if(Vizualizer::objs.find(glutGetWindow()) != Vizualizer::objs.end()){
		Vizualizer::objs[glutGetWindow()]->mouseClick(button, state, x, y);
	}
}

void rda::global_motion(int x, int y)
{
	if(Vizualizer::objs.find(glutGetWindow()) != Vizualizer::objs.end()){
		Vizualizer::objs[glutGetWindow()]->mouseMotion(x, y);
	}
}

void rda::global_reshape(int width, int heigth)
{		
	if(Vizualizer::objs.find(glutGetWindow()) != Vizualizer::objs.end()){
		Vizualizer::objs[glutGetWindow()]->reshape(width, heigth);
	}

}

void rda::global_key(unsigned char key, int x, int y)
{		
	if(Vizualizer::objs.find(glutGetWindow()) != Vizualizer::objs.end()){
		Vizualizer::objs[glutGetWindow()]->keyPress(key, x, y);
	}	
}




rda::Vizualizer::Vizualizer() 
{
	eyeX = 0.0;
	eyeY = 0.0;
	eyeZ = 1000.0;
	lastX = 0;
	lastY = 0;
}

void rda::Vizualizer::init(int* argc, char* argv[])
{
	glutInit(argc, argv);	
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);	
}

void rda::Vizualizer::start()
{
	if(glutGetWindow()){
		glutMainLoop();
	}
}

void rda::Vizualizer::createWindow(const char* title, int w, int h, int pos_x, int pos_y)
{	
	this->width_ = w;
	this->height_ = h;

	glutInitWindowSize(w, h);
	glutInitWindowPosition(pos_x, pos_y);
	this-> index = glutCreateWindow(title);	

	objs[this->index] = this;

	glClearColor(1, 1, 1, 0);
	/*glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0.0, w, h, 0.0, -1, 1);
	glMatrixMode(GL_MODELVIEW);*/
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	//Registration
	glutDisplayFunc(global_draw); 	
	glutReshapeFunc(global_reshape);
	//glutIdleFunc(global_draw);
	glutMouseFunc(global_mouse);
	glutMotionFunc(global_motion);
	glutKeyboardFunc(global_key);
}

void rda::Vizualizer::addCloud(rda::CloudPtr cloud, int mode, float r, float g, float b, float alpha, float pointSize)
{
	this->clouds_.push_back(cloud);
	this->modes_.push_back(mode);
	this->pointSizes_.push_back(pointSize);
	rda::Color c;
	c.r = r;
	c.g = g;
	c.b = b;
	c.alpha = alpha;
	this->colors_.push_back(c);
}

void rda::Vizualizer::addCloud(rda::Line line, int mode, float r, float g, float b, float alpha, float pointSize)
{
	rda::CloudPtr line_cloud(new rda::Cloud);
	line_cloud->push_back(line.start());
	line_cloud->push_back(line.end());

	this->clouds_.push_back(line_cloud);
	this->modes_.push_back(mode);
	this->pointSizes_.push_back(pointSize);
	rda::Color c;
	c.r = r;
	c.g = g;
	c.b = b;
	c.alpha = alpha;
	this->colors_.push_back(c);
}

void rda::Vizualizer::addCloud(rda::ApproximiedCloudPart& appr_cloud_part, int mode, float r, float g, float b, float alpha, float pointSize)
{
	rda::CloudPtr line_cloud(new rda::Cloud);
	line_cloud->push_back(appr_cloud_part.approx_line().start());
	line_cloud->push_back(appr_cloud_part.approx_line().end());

	this->clouds_.push_back(line_cloud);
	this->modes_.push_back(mode);
	this->pointSizes_.push_back(pointSize);
	rda::Color c;
	c.r = r;
	c.g = g;
	c.b = b;
	c.alpha = alpha;
	this->colors_.push_back(c);
}

void rda::Vizualizer::addCloud(rda::CloudPart& part, int mode, float r, float g, float b, float alpha, float pointSize)
{
	rda::CloudPtr part_cloud(new rda::Cloud);	

	for(int j = part.range().start; j <= part.range().end; j++){
		part_cloud->push_back(part.cloud()->points[j]);
	}

	this->clouds_.push_back(part_cloud);
	this->modes_.push_back(mode);
	this->pointSizes_.push_back(pointSize);

	rda::Color c;
	c.r = r;
	c.g = g;
	c.b = b;
	c.alpha = alpha;

	this->colors_.push_back(c);	
}

void rda::Vizualizer::addClouds(std::vector<rda::Line>& lines, int mode, float alpha, float pointSize)
{

	for(int i = 0; i < lines.size(); i++) {

		float r = (std::rand() % 100) / 100.0;
		float g = (std::rand() % 100) / 100.0;
		float b = (std::rand() % 100) / 100.0;

		addCloud(lines[i], mode, r, g, b, pointSize);
	}
}

void rda::Vizualizer::addClouds(std::vector<rda::ApproximiedCloudPart>& lines, int mode, float alpha, float pointSize)
{
	for(int i = 0; i < lines.size(); i++) {

		float r = (std::rand() % 50) / 100.0;
		float g = (std::rand() % 50) / 100.0;
		float b = (std::rand() % 50) / 100.0;

		addCloud(lines[i], mode, r, g, b, pointSize);
	}
}

void rda::Vizualizer::addClouds(std::vector<rda::CloudPtr>& clouds, int mode, float alpha, float pointSize)
{

	for(std::vector<rda::CloudPtr>::iterator it = clouds.begin(); it != clouds.end(); it++){

		float r = (std::rand() % 100) / 100.0;
		float g = (std::rand() % 100) / 100.0;
		float b = (std::rand() % 100) / 100.0;

		addCloud(*it, mode, r, g, b, alpha, pointSize);
	}

}

void rda::Vizualizer::addClouds(std::vector<rda::CloudPart>& clouds, int mode, float alpha, float pointSize)
{
	for(int i = 0; i < clouds.size(); i++){

		float r = (std::rand() % 100) / 100.0;
		float g = (std::rand() % 100) / 100.0;
		float b = (std::rand() % 100) / 100.0;

		this->addCloud(clouds[i], mode, r, g, b, alpha, pointSize);
	}
}

void rda::Vizualizer::mouseClick(int button, int state, int x, int y)
{
	switch(button){

		case GLUT_LEFT_BUTTON :
			if(state == GLUT_DOWN){
				lastX = x;
				lastY = y;				
			}
		break;
	}
}

void rda::Vizualizer::mouseMotion(int x, int y)
{	
	eyeX -= 0.005*(x - lastX)*eyeZ;
	eyeY += 0.005*(y - lastY)*eyeZ;

	lastX = x;
	lastY = y;
	draw(); // instead render loop

	//std::cout << eyeX << " " << eyeY << std::endl;
}

void rda::Vizualizer::keyPress(unsigned char key, int x, int y)
{
	double step = 30;
	if( (key == 'z') && (eyeZ + step < maxScale) ){
		eyeZ += step;
	}
	if( (key == 'x') && (eyeZ - step > 0.1) )
		eyeZ -= step;
	draw();
	//std::cout << "zoom:" << eyeZ << std::endl;
}

void rda::Vizualizer::reshape(int width, int heigth)
{
	if (heigth == 0){
		heigth = 1;	
	}
	width_ = width;
	height_ = heigth;
}

void drawCircle(double x, double y, double radius = 2.0)
{
	glBegin(GL_LINE_STRIP);
			   float a = 0;
			   int parts = 20;
               for(int i = 0; i <= parts; i++ ) {
                   a = (float)i / parts * 3.1415f * 2.0f;
                   glVertex2f(x + cos( a ) * radius, y + sin( a ) * radius );
               }
	glEnd();
}

void rda::Vizualizer::draw()
{	
	#pragma region Settings

	glClear(GL_COLOR_BUFFER_BIT);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	//glOrtho(0.0, this->width_, this->height_, 0.0, -1, 1);
	glViewport(0, 0, width_, height_);
	gluPerspective(45.0f, width_/height_, 0.1f, maxScale);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity(); 

	#pragma endregion

	gluLookAt(eyeX, eyeY, eyeZ, eyeX, eyeY, -1, 0, 1, 0);

	#pragma region Axes

	glColor3f(1.0f, 0.0f, 0.0f);
	glBegin(GL_LINES);
	glVertex3d(0, 0, 0);	
	glVertex3d(10, 0, 0);	
	glEnd();

	glColor3f(0.0f, 1.0f, 0.0f);
	glBegin(GL_LINES);
	glVertex3d(0, 0, 0);	
	glVertex3d(0, 10, 0);	
	glEnd();

	glColor3f(0.0f, 0.0f, 1.0f);
	glBegin(GL_LINES);
	glVertex3d(0, 0, 0);	
	glVertex3d(0, 0, 10);	
	glEnd();

	#pragma endregion	

	for(auto i = 0; i < clouds_.size(); i++){

		if(modes_[i] == rda::POINTS)
			glPointSize(pointSizes_[i]);
		else 
			glLineWidth(pointSizes_[i]);

		glColor4f(colors_[i].r, colors_[i].g, colors_[i].b, colors_[i].alpha);

		if(modes_[i] == rda::CIRCLES){
			for(pcl::PointCloud<pcl::PointXYZ>::iterator it = clouds_[i]->begin(); it != clouds_[i]->end(); ++it){				
				drawCircle(it->x, it->y, 4.0);
			}
		}
		else {

			glBegin(modes_[i]);
			for(pcl::PointCloud<pcl::PointXYZ>::iterator it = clouds_[i]->begin(); it != clouds_[i]->end(); ++it){
				glVertex3d(it->x, it->y, it->z);
			}
			glEnd();
		}
	}

	//rreturn to default
	glLineWidth(1.0f);
	glPointSize(1.0f);

	//glFlush();

	glutSwapBuffers();	
}

rda::Vizualizer::~Vizualizer()
{
	//TODO : find out how ot destroy vizualizer object while programm is running
	//now access exception occurs 
	objs.erase(this->index);
}
