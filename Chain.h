#ifndef CHAIN_H
#define CHAIN_H

class Vector;
class ChainElement;

class Chain {
public:
	double stiffness; // restoring force if chain is bent
	double stability; // restoring force if elements are pulled apart
	std::vector<ChainElement*> elements;
	Chain();
	void makeSimpleChain(int);
	void quickArrangementLine(Vector, Vector);
};

#endif