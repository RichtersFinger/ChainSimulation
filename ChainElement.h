#ifndef CHAINELEMENT_H
#define CHAINELEMENT_H
 
class ChainLink;

class ChainElement {
public:
	double x, y;
	double radius;
	bool active;
	ChainLink *linkL, *linkR;
	ChainElement();
	void linkLeftToElement(ChainElement *);
	void linkRightToElement(ChainElement *);
	~ChainElement();
};

#endif