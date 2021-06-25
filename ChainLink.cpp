#include <cstddef>
#include <math.h>
#include "ChainLink.h"
#include "ChainElement.h"

ChainLink::ChainLink() {
	linkLength0 = 0.0f;
	elA = NULL;
	elB = NULL;
}
ChainLink::ChainLink(ChainElement *someElementA, ChainElement *someElementB) {
	if (someElementA && someElementB)
		linkLength0 = sqrt((someElementA->x - someElementB->x)*(someElementA->x - someElementB->x)
				  + (someElementA->y - someElementB->y)*(someElementA->y - someElementB->y));
	else 
		linkLength0 = 0.0f;
	elA = someElementA;
	elB = someElementB;
}
void ChainLink::refreshLinkLength() {
	if (elA && elB)
		linkLength0 = sqrt((elA->x - elB->x)*(elA->x - elB->x)
				  + (elA->y - elB->y)*(elA->y - elB->y));
	else 
		linkLength0 = 0.0f;
}