#ifndef CHAIN_H
#define CHAIN_H

class Vector;
class ChainElement;

class Chain {
public:
	double stiffness, stiffnessdamping, stiffnessoffsetlength; // restoring force if chain is bent + damping; offset above which force is acting
	double stability, stabilitydamping; // restoring force if elements are pulled apart + damping
	std::vector<ChainElement*> elements;
	Chain();
	Chain(Chain *);
	void makeSimpleChain(int);
	void quickArrangementLine(Vector, Vector);
	void quickArrangementEntwined(Vector, double, int, double = 0.5);
	void refreshLinks();
	void loadFromChain(Chain *);
	void loadXYFromSTDVector(std::vector<Vector>);
	void loadVXYFromSTDVector(std::vector<Vector>);
	std::vector<Vector> XYto_STDVector();
	std::vector<Vector> VXYto_STDVector();
};

#endif
