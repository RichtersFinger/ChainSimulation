#ifndef WALL_H
#define WALL_H

#include <math.h>

enum WallType { wall, sphere };

class Wall {
public:
	double x, y; // left vertex
	double length, thickness; // length of the wall segment
	double normalx, normaly; // pointing away from wall
	double tangentx, tangenty; // direction of wall surface starting from x,y
	double stiffness, ndamping, tdamping, friction;
	WallType thisWallType;
	Wall() {
		x = 0.0;
		y = 0.0;
		length = 1.0;
		thickness = 0.04;
		normalx = 0.0;
		normaly = 1.0;
		tangentx = 1.0;
		tangenty = 0.0;
		stiffness = 2.0e6;
		ndamping = -2.0 * sqrt(stiffness * 3.1415926) * log(0.5)/sqrt(3.1415926*3.1415926 + log(0.5)*log(0.5));
		tdamping = -2.0 * sqrt(stiffness * 3.1415926) * log(0.8)/sqrt(3.1415926*3.1415926 + log(0.8)*log(0.8));
		friction = 0.2;
		thisWallType = wall;
	}
};

#endif
