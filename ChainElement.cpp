#include <cstddef>
#include "ChainElement.h"
#include "ChainLink.h"

ChainElement::ChainElement() {
	x = 0.0f;
	y = 0.0f;
	radius = 1.0f;
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