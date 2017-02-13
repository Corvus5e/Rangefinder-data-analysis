
#ifndef RDA_CLIENT_POINT_CLOUD_H
#define RDA_CLIENT_POINT_CLOUD_H

namespace client {	

	class PointCloud {

	private :

		int point_size_;
		double* data_; // represent points like [SIZE_OF_ARRAY, x, y, x, y, x, y, .... ]

	public:

		class iterator {

		private :
			double* x_pointer; // current x pointer
			int point_size_;

		public :

			iterator(double* curr_pointer, int point_size); // index of x // TODO thing how dont duplicate point_size

			bool operator == (iterator& op);

			bool operator != (iterator& op);

			iterator& operator ++ ();

			double x();

			double y();

			double z();
		};

		PointCloud(double* data, int point_size);

		iterator operator [] (int index);

		iterator begin();

		iterator end();

		int size();
	};

}

#endif
