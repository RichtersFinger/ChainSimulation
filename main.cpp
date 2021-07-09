/* g++ ChainElement.cpp ChainLink.cpp Chain.cpp Simulation.cpp main.cpp -std=c++11 -o run
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

#include "Vector.h"
#include "ChainLink.h"
#include "ChainElement.h"
#include "Wall.h"
#include "Chain.h"
#include "Simulation.h"

int main(int argc, char *argv[]){
	Simulation thisSimulation;

	thisSimulation.clearWalls();

	thisSimulation.addTrailofWalls(std::vector<Vector>{Vector(-100.0f, 2.0f), Vector(1.0f, 2.0f),
								Vector(1.0f, 2.0f), Vector(1.0f, 0.0f),
								Vector(1.0f, 0.0f), Vector(100.0f, 0.0f)});
	thisSimulation.writeWallsToFile("walls.dat");

	thisSimulation.initChain(500, 0.02f);

	//thisSimulation.chain.quickArrangementLine(Vector(0.0f, 2.5f), Vector(2.0f, 3.0f));
	thisSimulation.chain.quickArrangementEntwined(Vector(1.0f, 2.5f), 0.5f, 1);


	if (1 == 0) {
		thisSimulation.writeChainToFile("chain.dat");

		thisSimulation.step(3000);

		thisSimulation.writeChainToFile("chain2.dat");
	} else {
		for (int i = 0; i < 120; i++) {
			thisSimulation.writeChainToFile(i);
			thisSimulation.step(50);

		}

	}

	// next: implement collision with walls; then implement stiffness

	return 0;
}
