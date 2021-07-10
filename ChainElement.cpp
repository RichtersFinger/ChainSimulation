#include <cstddef>
#include "ChainElement.h"
#include "ChainLink.h"

ChainElement::ChainElement() {
	x = 0.0;
	y = 0.0;
	vx = 0.0;
	vy = 0.0;
	externalax = 0.0;
	externalay = 0.0;
	radius = 1.0;
	mass = 3.1415926;
	linkL = NULL;
	linkR = NULL;
	active = false;
}
void ChainElement::linkLeftToElement(ChainElement *someElement) {
	linkL = new ChainLink(someElement, this);
}
void ChainElement::linkRightToElement(ChainElement *someElement) {
	linkR = new ChainLink(this, someElement);
}
ChainElement::~ChainElement() {
	if (linkL) delete linkL;
	if (linkR) delete linkR;
}
