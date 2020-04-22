#include <iostream>
#include <limits>

using namespace std;

#ifndef LOWLEVEL_H
#define LOWLEVEL_H

///// common class definitions //////
struct Point{
	int x_pos, y_pos;
	Point(int xIn, int yIn): x_pos(xIn), y_pos(yIn){}
};

// defines a point on the gridmap
class State_map{

private:
	Point position;
	vector<double> h_val;

public:
	State_map(int numofgoalsIn);

	Point getPoint() const {return position;}
	vector<double> getH() const {return h_val;}
	void setPoint(Point positionIn){position = positionIn;}
	void setX(int x_posIn){position.x_pos = x_posIn;}
	void setY(int y_posIn){position.y_pos = y_posIn;}
	void setH(double* h_valIn){h_val = h_valIn;}
};



//// low-level specific definitions //////
void backDijkstra();

void unconstrainedSearch();



void constrainedSearch();

#endif