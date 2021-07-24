#include <cstddef>
#include <vector>
#include <math.h>
#include "Vector.h"
#include "ChainElement.h"
#include "ChainLink.h"
#include "Chain.h"
#include <iostream>

Chain::Chain() {
	stiffness = 4.0e4;
	stability = 2.0e4;
	stiffnessdamping = 0.0005 * sqrt(stiffness);
	stabilitydamping = 0.005 * sqrt(stability);
	stiffnessoffsetlength = 0.001;
}
Chain::Chain(Chain *somechain) {
	stiffness = 4.0e4;
	stability = 2.0e4;
	stiffnessdamping = 0.0005 * sqrt(stiffness);
	stabilitydamping = 0.005 * sqrt(stability);
	stiffnessoffsetlength = 0.001;

	// make chain elements
	for (int i = 0; i < somechain->elements.size(); i++) {
		ChainElement* someElement = new ChainElement();
		someElement->x = somechain->elements[i]->x;
		someElement->y = somechain->elements[i]->y;
		someElement->radius = somechain->elements[i]->radius;
		elements.push_back(someElement);
	}
	// make chain links
	for (int i = 0; i < somechain->elements.size(); i++) {
		if (i == 0)
			elements[i]->linkLeftToElement(NULL);
		else
			elements[i]->linkLeftToElement(elements[i-1]);
		if (i == somechain->elements.size() - 1)
			elements[i]->linkRightToElement(NULL);
		else
			elements[i]->linkRightToElement(elements[i+1]);
	}
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
	refreshLinks();
}
void Chain::quickArrangementEntwined(Vector centerVector, double radius, int nturns, double translation) {
	int ntotal = elements.size();
	int nperturn = ntotal/nturns;

	int ni = 0;
	for(const auto& value: elements) {
		value->x = centerVector.x + radius * cos(2.0 * 3.1415926 * float(ni)/float(nperturn));
		value->y = centerVector.y + radius * sin(2.0 * 3.1415926 * float(ni)/float(nperturn)) + translation * float(ni)/float(ntotal) * float(nturns) * radius;
		ni++;
	}
	refreshLinks();
}
void Chain::refreshLinks() {
	for(const auto& value: elements) {
		value->linkL->refreshLinkLength();
		value->linkR->refreshLinkLength();
	}
}
void Chain::loadFromChain(Chain *somechain) {
	for (int i = 0; i < elements.size(); i++) {
		elements[i]->x = somechain->elements[i]->x;
		elements[i]->y = somechain->elements[i]->y;
		elements[i]->vx = somechain->elements[i]->vx;
		elements[i]->vy = somechain->elements[i]->vy;
		elements[i]->radius = somechain->elements[i]->radius;
	}
}
void Chain::loadXYFromSTDVector(std::vector<Vector> somepoints) {
	for (int i = 0; i < somepoints.size(); i++) {
		elements[i]->x = somepoints[i].x;
		elements[i]->y = somepoints[i].y;
	}
}
void Chain::loadVXYFromSTDVector(std::vector<Vector> somepoints) {
	for (int i = 0; i < somepoints.size(); i++) {
		elements[i]->vx = somepoints[i].x;
		elements[i]->vy = somepoints[i].y;
	}
}
std::vector<Vector> Chain::XYto_STDVector() {
	std::vector<Vector> result;
	for(int i = 0; i < elements.size(); i++) {
		result.push_back(Vector(elements[i]->x, elements[i]->y));
	}
	return result;
}
std::vector<Vector> Chain::VXYto_STDVector() {
	std::vector<Vector> result;
	for(int i = 0; i < elements.size(); i++) {
		result.push_back(Vector(elements[i]->vx, elements[i]->vy));
	}
	return result;
}
