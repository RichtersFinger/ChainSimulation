#ifndef VECTOR_H
#define VECTOR_H
 
class Vector {
public:
  double x, y;
	Vector() {
		x = 0.0f;
		y = 0.0f;
	}
 	Vector(double x0, double y0) {
 		x = x0;
 		y = y0;
 	}
};

#endif