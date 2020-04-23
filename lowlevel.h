#include <iostream>
#include <math.h>
#include <limits>
#include <algorithm>
#include <vector>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <stack>
#include "param.h"

using namespace std;

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))
// #define NUMOFDIRS 9

#ifndef LOWLEVEL_H
#define LOWLEVEL_H

///// common class definitions //////
struct Point{
	int x_pos, y_pos;
	Point(){}
	Point(int xIn, int yIn): x_pos(xIn), y_pos(yIn){}
	bool operator==(const Point& rhs)
	{
	    return (x_pos==rhs.x_pos) && (y_pos==rhs.y_pos);
	}
};

struct Path{
	vector<Point> pathVect;
	double cost;
};

// defines a point on the gridmap
class State_map{

private:
	Point position;
	vector<double> h_vals;
	bool expanded;
	int goalIndex;

public:
	State_map(int numofgoalsIn);

	Point getPoint() const {return position;}
	vector<double> getH() const {return h_vals;}
	int getGoalIndex() const {return goalIndex;}
	bool isExpanded() const {return expanded;}
	void setPoint(Point positionIn){position = positionIn;}
	void setX(int x_posIn){position.x_pos = x_posIn;}
	void setY(int y_posIn){position.y_pos = y_posIn;}
	void setH(int index, double h_valIn){h_vals[index] = h_valIn;}
	void setGoalIndex(int goalIndexIn){goalIndex = goalIndexIn;}
	void expand(){expanded = true;}
	void contract(){expanded=false;}
};



//// low-level specific definitions //////

// compare struct for the priority queue
struct CompareF_map{
    bool operator()(State_map const & s1, State_map const & s2) {
        // return "true" if "p1" is ordered before "p2", for example:
        return s1.getH()[s1.getGoalIndex()] >  s2.getH()[s2.getGoalIndex()];
    }
};


void backDijkstra(vector<vector<State_map> >& gridmapIn, const vector<Point>& goals, double*map, 
	int x_size, int y_size, int collision_thresh);

vector<Path> unconstrainedSearch(const vector< vector<State_map> >& gridmapIn, const vector<Point>& robotPosnsIn, 
	const vector<int>& assignment, const vector<Point>& goalsIn, int x_size, int y_size);



void constrainedSearch();

#endif


// junk
vector<vector<int> > temp;
for(int i=0; i<numofagents; i++){

	vector<int> tempTemp;
	for(int j=0; j<numofgoals;j++){

		tempTemp.push_back( gridmap[robotPosns[i].y_pos - 1][robotPosns[i].x_pos - 1].getH()[j] );
	}
	temp.push_back(tempTemp);
}