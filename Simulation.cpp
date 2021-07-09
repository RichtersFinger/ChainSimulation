#include <vector>
#include <math.h>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include "Vector.h"
#include "ChainLink.h"
#include "ChainElement.h"
#include "Wall.h"
#include "Chain.h"
#include "Simulation.h"

Simulation::Simulation() {
	dt = 5.0e-4;
	time = 0.0f;
}
void Simulation::initChain(int nElements, double chainThickness) {
	chain.makeSimpleChain(nElements);
	for(int i = 0; i < chain.elements.size(); i++) {
		chain.elements[i]->radius = 0.5f * chainThickness;
	}
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

std::vector<Vector> Simulation::getInternalForces(Chain somechain) { // returns std:vector of accelerations; input is chain
	std::vector<Vector> result;
	for(int i = 0; i < somechain.elements.size(); i++) {
		Vector currentforce(0.0f, 0.0f);
		//somechain.
		//	double stiffness, stiffnessdamping; // restoring force if chain is bent + damping
		//	double stability, stabilitydamping; // restoring force if elements are pulled apart + damping

		// somechain.elements[i]->linkL
		// somechain.elements[i]->linkR


		// left link
		if (somechain.elements[i]->linkL->elA) {
			Vector direction = relativeVector(Vector(somechain.elements[i]->x, somechain.elements[i]->y), Vector(somechain.elements[i]->linkL->elA->x, somechain.elements[i]->linkL->elA->y));
			double distance = lengthOfVector(direction);
			if (distance != 0.0f) {
				direction.x /= distance;
				direction.y /= distance;

				Vector relativevelocity = relativeVector(Vector(somechain.elements[i]->vx, somechain.elements[i]->vy), Vector(somechain.elements[i]->linkL->elA->vx, somechain.elements[i]->linkL->elA->vy));

				double fx = somechain.stability * (distance - somechain.elements[i]->linkL->linkLength0) * direction.x + somechain.stabilitydamping * relativevelocity.x;
				double fy = somechain.stability * (distance - somechain.elements[i]->linkL->linkLength0) * direction.y + somechain.stabilitydamping * relativevelocity.y;

				currentforce.x += fx;
				currentforce.y += fy;
			}
		}
		// right link
		if (somechain.elements[i]->linkR->elB) {
			Vector direction = relativeVector(Vector(somechain.elements[i]->x, somechain.elements[i]->y), Vector(somechain.elements[i]->linkR->elB->x, somechain.elements[i]->linkR->elB->y));
			double distance = lengthOfVector(direction);
			if (distance != 0.0f) {
				direction.x /= distance;
				direction.y /= distance;

				Vector relativevelocity = relativeVector(Vector(somechain.elements[i]->vx, somechain.elements[i]->vy), Vector(somechain.elements[i]->linkR->elB->vx, somechain.elements[i]->linkR->elB->vy));

				double fx = somechain.stability * (distance - somechain.elements[i]->linkR->linkLength0) * direction.x + somechain.stabilitydamping * relativevelocity.x;
				double fy = somechain.stability * (distance - somechain.elements[i]->linkR->linkLength0) * direction.y + somechain.stabilitydamping * relativevelocity.y;

				currentforce.x += fx;
				currentforce.y += fy;
			}/**/
		}

		result.push_back(currentforce);
	}
	return result;
}
std::vector<Vector> Simulation::getWallForces(Chain somechain, std::vector<Wall*> somewalls) { // returns std:vector of accelerations; input is chain and walls
	std::vector<Vector> result;
	for(int i = 0; i < somechain.elements.size(); i++) {
		Vector currentforce(0.0f, 0.0f);
		for(const auto& somewall: somewalls) {
			double ndist = scalarproduct(Vector(somechain.elements[i]->x - somewall->x, somechain.elements[i]->y - somewall->y), Vector(somewall->normalx, somewall->normaly));
			if (ndist < somechain.elements[i]->radius && ndist > -somewall->thickness) {
				double tdist = scalarproduct(Vector(somechain.elements[i]->x - somewall->x, somechain.elements[i]->y - somewall->y), Vector(somewall->tangentx, somewall->tangenty));
				if (tdist > 0.0f && tdist < somewall->length) {
					// actual collision
					double vrelnorm = scalarproduct(Vector(somechain.elements[i]->vx, somechain.elements[i]->vy), Vector(somewall->normalx, somewall->normaly));
					double vreltang = scalarproduct(Vector(somechain.elements[i]->vx, somechain.elements[i]->vy), Vector(somewall->tangentx, somewall->tangenty));
					//std::max(
					double fnx = (somewall->stiffness * (somechain.elements[i]->radius - ndist) - somewall->damping * vrelnorm) * somewall->normalx;
					double fny = (somewall->stiffness * (somechain.elements[i]->radius - ndist) - somewall->damping * vrelnorm) * somewall->normaly;
					double fn = lengthOfVector(Vector(fnx, fny));
					double ftx = - somewall->damping * vreltang * somewall->tangentx;
					double fty = - somewall->damping * vreltang * somewall->tangenty;
					double ft = lengthOfVector(Vector(ftx, fty));

					double ftfrictionfactor;
					if (ft > 0.0f) {
						ftfrictionfactor = std::min(somewall->friction * fn, ft)/ft;
					}
					currentforce.x += fnx + ftfrictionfactor * ftx;
					currentforce.y += fny + ftfrictionfactor * fty;
				}
			}
		}
		result.push_back(currentforce);
	}
	return result;
}
std::vector<Vector> Simulation::getExternalForces(Chain somechain, double sometime) { // returns std:vector of accelerations; input is chain and time
	std::vector<Vector> result;
	for(int i = 0; i < somechain.elements.size(); i++) {
		result.push_back(Vector(0.0f, -9.81f));
	}
	return result;
}

void Simulation::step(int nsteps) {
	double time0 = time;
	Chain chain0(chain);
	std::vector<Vector> chain0_stdVector = chain0.XYto_STDVector();
	std::vector<Vector> chain_stdVector = chain.XYto_STDVector();
	std::vector<Vector> chainv0_stdVector = chain0.VXYto_STDVector();
	std::vector<Vector> chainv_stdVector = chain.VXYto_STDVector();
	//std::cout << chain0.elements[2]->x << " " << chain.elements[2]->x << std::endl;

	for (int i = 0; i < nsteps; i++) {
		std::vector<Vector> k1x = chainv_stdVector;
		std::vector<Vector> k1int = getInternalForces(chain);
		std::vector<Vector> k1walls = getWallForces(chain, walls);
		std::vector<Vector> k1ext = getExternalForces(chain, time);
		std::vector<Vector> k1v = addSTDVectorVectors(k1int, addSTDVectorVectors(k1walls, k1ext));
		chainv_stdVector = addSTDVectorVectors(chainv0_stdVector, productSTDVectorVectorScalar(k1v, 0.5f * dt));
		chain_stdVector = addSTDVectorVectors(chain0_stdVector, productSTDVectorVectorScalar(k1x, 0.5f * dt));
		time = time0 + ((double) i + 0.5f) * dt;
		chain.loadXYFromSTDVector(chain_stdVector);
		chain.loadVXYFromSTDVector(chainv_stdVector);

		std::vector<Vector> k2x = chainv_stdVector;
		std::vector<Vector> k2int = getInternalForces(chain);
		std::vector<Vector> k2walls = getWallForces(chain, walls);
		std::vector<Vector> k2ext = getExternalForces(chain, time);
		std::vector<Vector> k2v = addSTDVectorVectors(k2int, addSTDVectorVectors(k2walls, k2ext));
		chainv_stdVector = addSTDVectorVectors(chainv0_stdVector, productSTDVectorVectorScalar(k2v, 0.5f * dt));
		chain_stdVector = addSTDVectorVectors(chain0_stdVector, productSTDVectorVectorScalar(k2x, 0.5f * dt));
		time = time0 + ((double) i + 0.5f) * dt;
		chain.loadXYFromSTDVector(chain_stdVector);
		chain.loadVXYFromSTDVector(chainv_stdVector);

		std::vector<Vector> k3x = chainv_stdVector;
		std::vector<Vector> k3int = getInternalForces(chain);
		std::vector<Vector> k3walls = getWallForces(chain, walls);
		std::vector<Vector> k3ext = getExternalForces(chain, time);
		std::vector<Vector> k3v = addSTDVectorVectors(k3int, addSTDVectorVectors(k3walls, k3ext));
		chainv_stdVector = addSTDVectorVectors(chainv0_stdVector, productSTDVectorVectorScalar(k3v, 0.5f * dt));
		chain_stdVector = addSTDVectorVectors(chain0_stdVector, productSTDVectorVectorScalar(k3x, 0.5f * dt));
		time = time0 + ((double) i + 1.0f) * dt;
		chain.loadXYFromSTDVector(chain_stdVector);
		chain.loadVXYFromSTDVector(chainv_stdVector);

		std::vector<Vector> k4x = chainv_stdVector;
		std::vector<Vector> k4int = getInternalForces(chain);
		std::vector<Vector> k4walls = getWallForces(chain, walls);
		std::vector<Vector> k4ext = getExternalForces(chain, time);
		std::vector<Vector> k4v = addSTDVectorVectors(k4int, addSTDVectorVectors(k4walls, k4ext));

		chain_stdVector = addSTDVectorVectors(chain0_stdVector,
									productSTDVectorVectorScalar(addSTDVectorVectors(k1x,
																			addSTDVectorVectors(productSTDVectorVectorScalar(k2x, 2.0f),
																			addSTDVectorVectors(productSTDVectorVectorScalar(k3x, 2.0f),
																							k4x))),
									dt/6.0f));
		chainv_stdVector = addSTDVectorVectors(chainv0_stdVector,
									productSTDVectorVectorScalar(addSTDVectorVectors(k1v,
																			addSTDVectorVectors(productSTDVectorVectorScalar(k2v, 2.0f),
																			addSTDVectorVectors(productSTDVectorVectorScalar(k3v, 2.0f),
																							k4v))),
									dt/6.0f));
		chain.loadXYFromSTDVector(chain_stdVector);
		chain.loadVXYFromSTDVector(chainv_stdVector);

		if (i < nsteps - 1) {
			chain0_stdVector = chain_stdVector;
			chainv0_stdVector = chainv_stdVector;
		}
	}
	time = time0 + nsteps * dt;
}


void Simulation::writeWallsToFile(const char * filename) {
	std::ofstream outputfile;
	outputfile.open(filename);
	for (const auto& value: walls) {
		outputfile << value->x << "\t" << value->y << std::endl;
		outputfile << value->x + value->tangentx * value->length << "\t" << value->y + value->tangenty * value->length << std::endl;
		outputfile << std::endl;
	}
	outputfile.close();
}
void Simulation::writeChainToFile(const char * filename) {
	std::ofstream outputfile;
	outputfile.open(filename);
	for (const auto& value: chain.elements) {
		outputfile << value->x << "\t" << value->y << std::endl;
	}
	outputfile.close();
}
void Simulation::writeChainToFile(int number) {
	std::ofstream outputfile;
	std::ostringstream filename;

	std::stringstream ss;
	ss << "data/output" << std::setfill('0') << std::setw(5) << number << ".dat";
	outputfile.open(ss.str());
	for (const auto& value: chain.elements) {
		outputfile << value->x << "\t" << value->y << std::endl;
	}
	outputfile.close();

	return;
}

std::vector<Vector> Simulation::addSTDVectorVectors( std::vector<Vector> vecA, std::vector<Vector> vecB) {
	std::vector<Vector> result;
	for(int i = 0; i < vecA.size(); i++) {
		result.push_back(Vector(vecA[i].x + vecB[i].x, vecA[i].y + vecB[i].y));
	}
	return result;
}

std::vector<Vector> Simulation::productSTDVectorVectorScalar( std::vector<Vector> vecA, double factor) {
	std::vector<Vector> result;
	for(int i = 0; i < vecA.size(); i++) {
		result.push_back(Vector(factor * vecA[i].x, factor * vecA[i].y));
	}
	return result;
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
double Simulation::scalarproduct(Vector somePointA, Vector somePointB) { // from A to B
	return somePointB.x * somePointA.x + somePointB.y * somePointA.y;
}
