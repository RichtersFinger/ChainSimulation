#include <cstddef>
#include <vector>
#include <math.h>
#include "Vector.h"
#include "ChainElement.h"
#include "ChainLink.h"
#include "Chain.h"
#include <iostream>

Chain::Chain() {
	stiffness = 100.0f;
	stability = 100.0f;
}
void Chain::makeSimpleChain(int nElements) {
	// destroy previous chain
	for(const auto& value: elements) {
		delete value;
	}
	elements.clear();

	// make chain elements
	for (int i = 0; i < nElements; i++) {
		ChainElement* someElement = new ChainElement();
		elements.push_back(someElement);
	}
	// make chain links
	for (int i = 0; i < nElements; i++) {
		if (i == 0)
			elements[i]->linkLeftToElement(NULL);
		else
			elements[i]->linkLeftToElement(elements[i-1]);
		if (i == nElements - 1)
			elements[i]->linkRightToElement(NULL);
		else
			elements[i]->linkRightToElement(elements[i+1]);
	}
}
void Chain::quickArrangementLine(Vector startVector, Vector endVector) {
	double distance = sqrt((startVector.x - endVector.x)*(startVector.x - endVector.x) 
				+ (startVector.y - endVector.y)*(startVector.y - endVector.y));
	Vector direction((endVector.x - startVector.x)/distance, (endVector.y - startVector.y)/distance);
	int ni = 0;
	int ntotal = elements.size();
	for(const auto& value: elements) {
		value->x = startVector.x + distance * direction.x/float(ntotal-1)*float(ni);
		value->y = startVector.y + distance * direction.y/float(ntotal-1)*float(ni);
		ni++;
	}
}