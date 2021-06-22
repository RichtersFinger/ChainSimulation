#ifndef SIMULATION_H
#define SIMULATION_H
 
class Simulation {
public:
	Chain chain;
	std::vector<Wall*> walls;
	Simulation();
	void initChain(int);
	void addTrailofWalls(std::vector<Vector>);
	void clearWalls();
	void writeWallsToFile(const char *);
	void writeChainToFile(const char *);

private:
	 double lengthOfVector(Vector);
	 double distanceOfPoints(Vector, Vector);
	 Vector relativeVector(Vector, Vector);
};

#endif