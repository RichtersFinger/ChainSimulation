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

	/*
	thisSimulation.addTrailofWalls(std::vector<Vector>{Vector(-100.0, 2.0), Vector(1.0, 2.0),
									Vector(1.0, 2.0), Vector(1.0, 0.0),
									Vector(1.0, 0.0), Vector(100.0, 0.0)});
	*/
	thisSimulation.addTrailofWalls(std::vector<Vector>{Vector(-100.0, 2.0), Vector(0.9, 2.0),
								Vector(0.9, 2.0), Vector(1.0, 1.9)});
								//, Vector(1.0, 1.9), Vector(1.0, 0.0)
	thisSimulation.writeWallsToFile("walls.dat");

	thisSimulation.initChain(200, 0.01, 8.0e4, 0.001);

	//thisSimulation.chain.quickArrangementLine(Vector(0.0f, 2.5f), Vector(2.0f, 3.0f));
	thisSimulation.chain.quickArrangementEntwined(Vector(0.0, 2.5), 0.5, 2);


	if (1 == 0) {
		thisSimulation.writeChainToFile("chain.dat");

		thisSimulation.step(3000);

		thisSimulation.writeChainToFile("chain2.dat");
	} else {
		for (int i = 0; i < 180; i++) {
			//thisSimulation.writeChainToFile(i);
			thisSimulation.step(50);
		}
		thisSimulation.chain.elements[0]->externalax = 300.0;
		thisSimulation.chain.elements[0]->externalay = 300.0;
		for (int i = 0; i < 180; i++) {
			if (thisSimulation.chain.elements[0]->x > 1.2) {
				thisSimulation.chain.elements[0]->externalax = 0.0;
				thisSimulation.chain.elements[0]->externalay = 0.0;
			}
			thisSimulation.writeChainToFile(i);
			thisSimulation.step(50);
		}

	}

	// next: implement collision with walls; then implement stiffness

	return 0;
}
