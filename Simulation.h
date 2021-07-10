#ifndef SIMULATION_H
#define SIMULATION_H

class Simulation {
public:
	Chain chain;
	std::vector<Wall*> walls;
	double time;
	double dt;

	Simulation();
	void initChain(int, double, double = 1.0e4, double = 0.1);
	void addTrailofWalls(std::vector<Vector>);
	void clearWalls();
	void writeWallsToFile(const char *);
	void writeChainToFile(const char *);
	void writeChainToFile(int);
	void step(int);

private:
	 double lengthOfVector(Vector);
	 double distanceOfPoints(Vector, Vector);
	 Vector relativeVector(Vector, Vector);
	 double scalarproduct(Vector, Vector);
	 std::vector<Vector> addSTDVectorVectors( std::vector<Vector>, std::vector<Vector>);
	 std::vector<Vector> productSTDVectorVectorScalar( std::vector<Vector>, double);
	 std::vector<Vector> getInternalForces(Chain); // returns std:vector of accelerations; input is chain
	 std::vector<Vector> getWallForces(Chain, std::vector<Wall*>); // returns std:vector of accelerations; input is chain and walls
	 std::vector<Vector> getExternalForces(Chain, double); // returns std:vector of accelerations; input is chain and time
};

#endif
