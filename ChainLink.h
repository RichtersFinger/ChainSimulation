#ifndef CHAINLINK_H
#define CHAINLINK_H

class ChainElement;

class ChainLink {
public:
	ChainElement *elA, *elB;
	double linkLength0;
	ChainLink();
	ChainLink(ChainElement *, ChainElement *);
};

#endif