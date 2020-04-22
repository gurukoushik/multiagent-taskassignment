#include <iostream>

#ifndef LOWLEVEL_H
#define LOWLEVEL_H

// common class definitions
struct Point{
	int x_index, y_index;

	Point(int xIn, int yIn): x_index(xIn), y_index(yIn){}
};

class State_map{
private:

	Point position;
	double h_val;

public:
	State_map(){}

	Point getPoint() const {return position;}
	double getH() const {return h_val;}
	void setPoint(Point positionIn){position = positionIn;}
	void setH(double h_valIn){h_val = h_valIn;}
};



// low-level specific definitions
void unconstrainedSearch();



void constrainedSearch();

#endif