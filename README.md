
# Basic Chain Simulation

Compile using for example
   ```console
   $ g++ ChainElement.cpp ChainLink.cpp Chain.cpp Simulation.cpp main.cpp -std=c++11 -o run -O3
   ```
   
The output data is written to the subdirectory `data/`. Use for example the provided gnuplot script `plot.plt` to plot the results.

See also the [demonstration](https://youtu.be/O_HGXQ6sR9I) for an example application for the simulation of the chain fountain or Mould effect as well as illustration of the model used here.

## Simulation Setup
The setup of a simulation consists of two steps, configuring the boundaries (walls and other obstacles) and the chain itself. At first a simulation object has to be created,
```
Simulation thisSimulation;
```
This object stores all information on a single simulation. 

* **boundary**: There are two types of boundaries currently implemented. The walls are defined as segments with a start and end point. Geometric points are generally represented as `Vector` objects like
```
Vector thisVector(1.0, 1.0);
```
Furthermore, walls have an orientation associated with them. This orientation or surface normal is defined to point in a 90° angle (i.e. to the left) relative to the relative vector from start to end point of the wall. All wall-type boundaries are collected in the member `walls` of the simulation object, a `std::vector<Wall*>`. The procedure `addTrailofWalls(..)` can be used to comfortably introduce new walls into the system. Its argument is a `std::vector<Vector>` with alternating start and end points, e.g.
```
thisSimulation.addTrailofWalls(std::vector<Vector>{
	Vector(0.0, 0.0), Vector(1.0, 0.0), // wall segment 1 from (0,0) to (1,0)
	Vector(1.0, 0.0), Vector(1.0, 1.0) // wall segment 2 from (1,0) to (1,1)
});
```
The second option for creating boundary objects is the routine `addSphericalBoundary(..)` again for the simulation object in order to create a spherical boundary. The three required arguments are the x- and y-coordinates as well as the radius.

* **chain**: The chain object for the simulation can be created by using for example
```
thisSimulation.initChain(numberOfSamples, chainThickness, chainStiffness, chainStiffnessOffset, chainStability);
```
Here, the parameters `numberOfSamples` denotes the total number of samples (or depending on the application the number of beads) used to simulate the chain, `chainThickness` the chain's cross section diameter/bead diameter, `chainStiffness` the strength of the resistance to being bent, `chainStiffnessOffset` the offset for the chain's stiffness allowing some amount of bending without internal forces, and `chainStability` the strength of the resistance to being pulled apart. While this sets up some of the internal data of the simulation object, it is required that the equilibrium state of the chain is defined as well (basically the distance between samples in the chain). This can be done by for example the procedure 
```
thisSimulation.chain.quickArrangementEntwined(Vector(startX, startY), thisRadius, thisLoops, thisTranslation);
```
Using this the chain is first arranged in a two-dimensional helix shape starting at `Vector(startX, startY)` and with radius `thisRadius` and `thisLoops` number of loops as well as a rate of translation `thisTranslation`. Afterwards the relative distances between chain samples are stored as the chain's equilibrium position.

## Execution
Using the method `step(..)` with the argument of the number of time steps (time step length is stored in the variable `dt` of the simulation object) on the simulation allows to advance the system in time. Use for example
```
thisSimulation.step(floor(1.0/60.0/thisSimulation.dt));
```
to perform time steps roughly equivalent to one 60th of a second. The numerical integration is done with a Runge-Kutta algorithm of fourth order. Inbetween calls to `step(..)` a sample-specific external acceleration/force can be configured by setting for example
```
thisSimulation.chain.elements[sampleIndex]->externalax = 9.81;
```
resulting in the sample `sampleIndex` (counting starts at zero) experiencing an external force equal to gravity but in the positive x-direction.

## In- and Output
A simulation object allows to write its current state to a data file. First, the boundary elements created initially can be written to a file using 
```
thisSimulation.writeWallsToFile("thisBoundary.dat");
```
The written file contains the x- and y-coordinates of wall segments as well as sampled spherical boundary elements such that it can easily be plotted with any common graphing utility like gnuplot. The chain itself can be written to a file by invoking 
```
thisSimulation.writeChainToFile(someIndex);
```
This procedure writes the x- and y-coordinates of all chain samples to a file in the `data/` subdirectory in the format `outputXXXXX.dat` where the blanks are filled with the 5-digit decimal representation of `someIndex`.
Lastly, the current state of a chain can be written to a file using a different format suitable for a later restoration of a system state. Use
```
thisSimulation.saveChainBackup("thisChainBackup.dat");
```
to write the backup file. Restoring a specific system state requires that the system setup is performed in a compatible manner. This means that the chain has to be initialized with `initChain(..)` and the previous equilibrium distances between samples have to be configured indentically (using for example the same arguments for a call to `quickArrangementEntwined(..)`.