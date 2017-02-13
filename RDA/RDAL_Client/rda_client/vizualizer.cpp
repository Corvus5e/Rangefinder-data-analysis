#include <iostream>
#include <vector>
#include <map>

#include "vizualizer.h"
#include "GL\glut.h"

std::map<int, client::Vizualizer*> objs; // id of gl windows -> pointer to Vizualizer object

void client::global_draw(){		
	if(objs.find(glutGetWindow()) != objs.end()){
		objs[glutGetWindow()]->draw();
	}
}

void client::global_mouse(int button, int state, int x, int y){
	if(objs.find(glutGetWindow()) != objs.end()){
		objs[glutGetWindow()]->mouseClick(button, state, x, y);
	}
}

void client::global_motion(int x, int y){
	if(objs.find(glutGetWindow()) != objs.end()){
		objs[glutGetWindow()]->mouseMotion(x, y);
	}
}

void client::global_reshape(int width, int heigth){		
	if(objs.find(glutGetWindow()) != objs.end()){
		objs[glutGetWindow()]->reshape(width, heigth);		
	}

}

void client::global_key(unsigned char key, int x, int y){		
	if(objs.find(glutGetWindow()) != objs.end()){
		objs[glutGetWindow()]->keyPress(key, x, y);				
	}	
}




client::Vizualizer::Vizualizer() {
	eyeX = 0.0;
	eyeY = 0.0;
	eyeZ = 2.0;
	lastX = 0;
	lastY = 0;
}

void client::Vizualizer::init(int* argc, char* argv[]) {
	glutInit(argc, argv);	
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
}

void client::Vizualizer::start(){
	if(glutGetWindow()){
		glutMainLoop();
	}
}

void client::Vizualizer::createWindow(const char* title, int w, int h, int x, int y) {
	
	this->width_ = w;
	this->height_ = h;

	glutInitWindowSize(w, h);
	glutInitWindowPosition(x, y);
	this-> index = glutCreateWindow(title);	

	objs[this->index] = this;

	glClearColor(0, 0, 0, 0);
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

void client::Vizualizer::addCloud(client::PointCloud cloud, int mode, float r, float g, float b, float alpha, float pointSize){
	this->clouds_.push_back(cloud);
	this->modes_.push_back(mode);
	this->pointSizes_.push_back(pointSize);
	client::Color c;
	c.r = r;
	c.g = g;
	c.b = b;
	c.alpha = alpha;
	this->colors_.push_back(c);
}

void client::Vizualizer::addClouds(std::vector<client::PointCloud>& clouds, int mode, float alpha, float pointSize){

	for(std::vector<client::PointCloud>::iterator it = clouds.begin(); it != clouds.end(); it++){

		float r = (std::rand() % 100) / 100.0;
		float g = (std::rand() % 100) / 100.0;
		float b = (std::rand() % 100) / 100.0;

		addCloud(*it, mode, r, g, b, alpha, pointSize);
	}

}

void client::Vizualizer::mouseClick(int button, int state, int x, int y){

	switch(button){

		case GLUT_LEFT_BUTTON :
			if(state == GLUT_DOWN){
				lastX = x;
				lastY = y;
				//std::cout << x << " " << y << std::endl;
			}
		break;
	}
}

void client::Vizualizer::mouseMotion(int x, int y){	

	eyeX -= 0.005*(x - lastX)*eyeZ;
	eyeY += 0.005*(y - lastY)*eyeZ;

	lastX = x;
	lastY = y;
	draw(); // instead render loop (yet )

	//std::cout << eyeX << " " << eyeY << std::endl;
}

void client::Vizualizer::keyPress(unsigned char key, int x, int y){
	double step = 3.5;
	if( (key == 'z') && (eyeZ + step < maxScale) ){
		eyeZ += step;
	}
	if( (key == 'x') && (eyeZ - step > 0.1) )
		eyeZ -= step;
	draw();
	//std::cout << key << " " << x << " " << y <<  std::endl;
}

void client::Vizualizer::reshape(int width, int heigth){
	if (heigth == 0){
		heigth = 1;	
	}
	width_ = width;
	height_ = heigth;
}

void client::Vizualizer::draw() {
	
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

	for(int i = 0; i < clouds_.size(); i++){

		if(modes_[i] == client::POINTS)
			glPointSize(pointSizes_[i]);
		else 
			glLineWidth(pointSizes_[i]);

		glColor4f(colors_[i].r, colors_[i].g, colors_[i].b, colors_[i].alpha);		
		glBegin(modes_[i]);

		for(client::PointCloud::iterator it = clouds_[i].begin(); it != clouds_[i].end(); ++it){
			glVertex3d(it.x() / 20, it.y()/20, 0);
		}

		glEnd();
	}

	//rreturn to default
	glLineWidth(1.0f);
	glPointSize(1.0f);

	//glFlush();

	glutSwapBuffers();	
}

client::Vizualizer::~Vizualizer() {
	//TODO : find out how ot destroy vizualizer object while programm is running
	//now access exception occurs 
	objs.erase(this->index);
}