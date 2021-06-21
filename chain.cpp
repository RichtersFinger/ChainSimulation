/* g++ chain.cpp -std=c++11 -o run
 */

#include <vector>
#include <iostream>
#include <sstream>
#include <fstream>
#include <stdio.h>
#include <iomanip> // precision //
#include <string.h>
#include <math.h>
#include <cstdlib> // random numbers //
#include <time.h> // for 'random' seed //

const int N = 10;
const double PI = 3.1415926535f;

class ChainLink;
class ChainElement;
class Chain;
class Wall;
class Simulation;

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
class ChainLink {
public:
	ChainElement *elA, *elB;
	ChainLink() {
		elA = NULL;
		elB = NULL;
	}
	ChainLink(ChainElement *someElementA, ChainElement *someElementB) {
		elA = someElementA;
		elB = someElementB;
	}
};
class ChainElement {
  public:
    double x, y;
	 double radius;
	 bool active;
	 ChainLink *linkL, *linkR;
    ChainElement() {
      x = 0.0f;
      y = 0.0f;
		radius = 1.0f;
		linkL = NULL;
		linkR = NULL;
		active = false;
    }
	 void linkLeftToElement(ChainElement *someElement) {
		 linkL = new ChainLink(someElement, this);
	 }
	 void linkRightToElement(ChainElement *someElement) {
		 linkR = new ChainLink(this, someElement);
	 }
	 ~ChainElement() {
		   if (linkL) delete linkL;
		   if (linkR) delete linkR;
	 }
};
class Chain {
  public:
	 double stiffness; // restoring force if chain is bent
	 double stability; // restoring force if elements are pulled apart
	 double lengthPerElement0;
    std::vector<ChainElement*> elements;
    Chain() {
		 stiffness = 100.0f;
		 stability = 100.0f;
		 lengthPerElement0 = 0.1f;
    }
	 void makeChain(int nelements) {
		// destroy previous chain
		for(const auto& value: elements) {
			delete value;
		}
		elements.clear();

		// make chain elements
		for (int i = 0; i < nelements; i++) {
			ChainElement* someElement = new ChainElement();
			elements.push_back(someElement);
		}
		// make chain links
		for (int i = 0; i < nelements; i++) {
			if (i == 0)
				elements[i]->linkLeftToElement(NULL);
			else
				elements[i]->linkLeftToElement(elements[i-1]);
			if (i == nelements - 1)
				elements[i]->linkRightToElement(NULL);
			else
				elements[i]->linkRightToElement(elements[i+1]);
		}
	 }
};

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

class Simulation {
  public:
	 std::vector<Wall*> walls;
    Simulation() {
    }
	 double lengthOfVector(Vector someVector) {
		 return sqrt(someVector.x*someVector.x + someVector.y*someVector.y);
	 }
	 double distanceOfPoints(Vector somePointA, Vector somePointB) {
		 return lengthOfVector(relativeVector(somePointA, somePointB));
	 }
	 Vector relativeVector(Vector somePointA, Vector somePointB) { // from A to B
		 return Vector(somePointB.x - somePointA.x, somePointB.y - somePointA.y);
	 }
	 void addTrailofWalls(std::vector<Vector> trailOfPoints ) {
		 int nWallSegments = trailOfPoints.size();
		 for (int i = 0; i < nWallSegments; i+=2) {
			if (i + 1 < nWallSegments) {
				Wall* thisWall = new Wall();

				thisWall->x = trailOfPoints[i].x;
				thisWall->y = trailOfPoints[i].y;
				thisWall->length = distanceOfPoints(trailOfPoints[i], trailOfPoints[i + 1]);
				Vector thisTangent = relativeVector(trailOfPoints[i], trailOfPoints[i + 1]);
				thisWall->tangentx = thisTangent.x/thisWall->length;
				thisWall->tangenty = thisTangent.y/thisWall->length;
				thisWall->normalx = -thisWall->tangenty;
				thisWall->normaly = thisWall->tangentx;

			 	walls.push_back(thisWall);
			}
		 }
	 }
	 void clearWalls() {
		 for(const auto& value: walls) {
			 delete value;
		 }
		 walls.clear();
	 }
	 void writeWallsToFile(const char * filename) {
	 	std::ofstream outputfile;
	 	outputfile.open(filename);
		for(const auto& value: walls) {
			outputfile << value->x << "\t" << value->y << std::endl;
			outputfile << value->x + value->tangentx * value->length << "\t" << value->y + value->tangenty * value->length << std::endl;
			outputfile << std::endl;
		}
		outputfile.close();
	 }

};

int main(int argc, char *argv[]){

	Simulation thisSimulation;

	thisSimulation.clearWalls();

	thisSimulation.addTrailofWalls(std::vector<Vector>{Vector(0.0f, 2.0f), Vector(1.0f, 2.0f), Vector(1.0f, 2.0f), Vector(1.0f, 0.0f)});

	thisSimulation.writeWallsToFile("walls.dat");
	return 0;
}
