#ifndef WALL_H
#define WALL_H

#include <math.h>

class Wall {
public:
	double x, y; // left vertex
	double length, thickness; // length of the wall segment
	double normalx, normaly; // pointing away from wall
	double tangentx, tangenty; // direction of wall surface starting from x,y
	double stiffness, damping, friction;
	Wall() {
		x = 0.0f;
		y = 0.0f;
		length = 1.0f;
		thickness = 0.005f;
		normalx = 0.0f;
		normaly = 1.0f;
		tangentx = 1.0f;
		tangenty = 0.0f;
		stiffness = 5.0e5;
		damping = -2.0f * sqrt(stiffness * 3.1415926f) * log(0.5f)/sqrt(3.1415926f*3.1415926f + log(0.5f)*log(0.5f)); // missing factor of sqrt(mass) of element
		friction = 0.5f;
	}
};

#endif
