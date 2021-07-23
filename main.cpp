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

	// setup container
	double containerposx = 0.0;
	double containerposy = 0.0;
	double containerheight = 1.0;
	double containerwallwidth = 0.1;
	double containerwidth = 0.6;
	double dropheight = 10.0;
	thisSimulation.addTrailofWalls(std::vector<Vector>{Vector(containerposx, containerposy + containerheight), Vector(containerposx, containerposy), // left wall
								Vector(containerposx, containerposy), Vector(containerposx + containerwidth, containerposy), // floor
								Vector(containerposx + containerwidth, containerposy), Vector(containerposx + containerwidth, containerposy + containerheight), // right wall
								Vector(containerposx + containerwidth + containerwallwidth, containerposy + containerheight), Vector(containerposx + containerwidth + containerwallwidth, containerposy + containerheight - dropheight), // right wall outer

								Vector(0.0, containerposy + containerheight - dropheight), Vector(8.0, containerposy + containerheight - dropheight)});
	// change this to account for fast impact of chain
	thisSimulation.walls.back()->thickness = 1.0;
	// rounded off tips for container walls
	thisSimulation.addSphericalBoundary(containerposx + containerwidth + 0.5*containerwallwidth, containerposy + containerheight, 0.5*containerwallwidth);
	thisSimulation.addSphericalBoundary(containerposx - 0.5*containerwallwidth, containerposy + containerheight, 0.5*containerwallwidth);
	thisSimulation.writeWallsToFile("walls.dat");

	// make chain
	thisSimulation.initChain(8*200, 0.01, 5.0e4, 0.5, 2.0e4);

	// arrange chain elements in a loop above the container
	thisSimulation.chain.quickArrangementEntwined(Vector(containerposx + 0.5 * containerwidth, containerposy + 0.5 * containerheight), 0.33 * containerwidth, 4*4);

	// load file or integrate drop
	if (false) {
		thisSimulation.loadChainBackup("chainbackup.dat");
	} else {
		for (int i = 0; i < 15; i++) {
			thisSimulation.writeChainToFile(0);
			thisSimulation.step(floor(0.1/thisSimulation.dt));
			std::cout << i << std::endl;
		}
		thisSimulation.saveChainBackup("chainbackup.dat");
	}
	// pull first element out of container
	thisSimulation.chain.elements[0]->externalax = 400.0;
	thisSimulation.chain.elements[0]->externalay = 1200.0;
	std::cout << "start" << std::endl;
	for (int i = 0; i < 180; i++) {
		std::cout << i << std::endl;
		if (thisSimulation.chain.elements[0]->y > containerposy + containerheight + 2.0 && thisSimulation.chain.elements[0]->x > containerposx + containerwidth + containerwallwidth + 0.5) {
			thisSimulation.chain.elements[0]->externalax = 0.0;
			thisSimulation.chain.elements[0]->externalay = 0.0;
		}
		thisSimulation.writeChainToFile(0);
		thisSimulation.writeChainToFile(i);
		thisSimulation.step(floor(0.1/thisSimulation.dt));
	}

	return 0;
}
