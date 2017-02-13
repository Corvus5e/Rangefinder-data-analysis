
#include "point_cloud.h"

client::PointCloud::iterator::iterator(double* curr_p, int point_size){	
	x_pointer = curr_p;
	this->point_size_ = point_size;
}

double client::PointCloud::iterator::x() {
	return *(x_pointer );
}

double client::PointCloud::iterator::y() {
	return *(x_pointer + 1);
}

double client::PointCloud::iterator::z() {
	return *(x_pointer + 2);
}

bool client::PointCloud::iterator::operator==(client::PointCloud::iterator& op){
	return op.x_pointer == this->x_pointer;
}

bool client::PointCloud::iterator::operator!=(client::PointCloud::iterator& op){
	return op.x_pointer != this->x_pointer;
}

 client::PointCloud::iterator& client::PointCloud::iterator::operator++(){
	 this->x_pointer += this->point_size_;
	 return *this;
}

client::PointCloud::PointCloud(double* data, int point_size){
	data_ = data;
	point_size_ = point_size;
}

client::PointCloud::iterator client::PointCloud::operator[](int index){
	return client::PointCloud::iterator( &data_[ index*point_size_ + 1 ], this->point_size_ );
}

client::PointCloud::iterator client::PointCloud::begin(){	
	return client::PointCloud::iterator( &data_[1], this->point_size_ );
}

client::PointCloud::iterator client::PointCloud::end(){
	return client::PointCloud::iterator( &data_[ (int)data_[0] ], this->point_size_ ); // XD
}

int client::PointCloud::size() {
	return (data_[0] - 1) / point_size_;
}
