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
		x = 0.0;
		y = 0.0;
		length = 1.0;
		thickness = 0.005;
		normalx = 0.0;
		normaly = 1.0;
		tangentx = 1.0;
		tangenty = 0.0;
		stiffness = 5.0e5;
		damping = -2.0 * sqrt(stiffness * 3.1415926) * log(0.5f)/sqrt(3.1415926*3.1415926 + log(0.5)*log(0.5)); // missing factor of sqrt(mass) of element
		friction = 0.5;
	}
};

#endif
