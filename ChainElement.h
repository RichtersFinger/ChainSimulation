#ifndef CHAINELEMENT_H
#define CHAINELEMENT_H

class ChainLink;

class ChainElement {
public:
	double x, y;
	double vx, vy;
	double externalax, externalay;
	double radius;
	double mass;
	bool active;
	ChainLink *linkL, *linkR;
	ChainElement();
	void linkLeftToElement(ChainElement *);
	void linkRightToElement(ChainElement *);
	~ChainElement();
};

#endif
