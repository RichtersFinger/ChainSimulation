#include <vector>
#include <math.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include "Vector.h"
#include "ChainLink.h"
#include "ChainElement.h"
#include "Wall.h"
#include "Chain.h"
#include "Simulation.h"

Simulation::Simulation() {
}
void Simulation::initChain(int nElements) {
	chain.makeSimpleChain(nElements);
}
void Simulation::addTrailofWalls(std::vector<Vector> trailOfPoints ) {
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
void Simulation::clearWalls() {
	for(const auto& value: walls) {
		delete value;
	}
	walls.clear();
}
void Simulation::writeWallsToFile(const char * filename) {
	std::ofstream outputfile;
	outputfile.open(filename);
	for(const auto& value: walls) {
		outputfile << value->x << "\t" << value->y << std::endl;
		outputfile << value->x + value->tangentx * value->length << "\t" << value->y + value->tangenty * value->length << std::endl;
		outputfile << std::endl;
	}
	outputfile.close();
}
void Simulation::writeChainToFile(const char * filename) {
	std::ofstream outputfile;
	outputfile.open(filename);
	for(const auto& value: chain.elements) {
		outputfile << value->x << "\t" << value->y << std::endl;
	}
	outputfile.close();
}

double Simulation::lengthOfVector(Vector someVector) {
	return sqrt(someVector.x*someVector.x + someVector.y*someVector.y);
}
double Simulation::distanceOfPoints(Vector somePointA, Vector somePointB) {
	return lengthOfVector(relativeVector(somePointA, somePointB));
}
Vector Simulation::relativeVector(Vector somePointA, Vector somePointB) { // from A to B
	return Vector(somePointB.x - somePointA.x, somePointB.y - somePointA.y);
}