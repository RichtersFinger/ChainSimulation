#ifndef WALL_H
#define WALL_H

class Wall {
public:
	double x, y; // left vertex
	double length, thickness; // length of the wall segment
	double normalx, normaly; // pointing away from wall
	double tangentx, tangenty; // direction of wall surface starting from x,y
	Wall() {
		x = 0.0f;
		y = 0.0f;
		length = 1.0f;
		thickness = 0.1f;
		normalx = 0.0f;
		normaly = 1.0f;
		tangentx = 1.0f;
		tangenty = 0.0f;
	}
};

#endif