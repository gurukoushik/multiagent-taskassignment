#include "lowlevel.h"

///// class definitions ////////
State_map::State_map(int numofgoalsIn){
	
	for(int i=0; i<numofgoalsIn; i++){

		h_val.push_back(numeric_limits<double>::infinity());
	}
}

void backDijkstra(vector<vector<State_map> >& gridmapIn, const vector<Point>& goal, int x_size, int y_size){

	gridmap[goal.y - 1][goal.x - 1].setH(0.0);

	// populate map
	for (int i =0; i<y_size; i++){
		for (int j=0; j<x_size; j++){

			grid_map[i][j].setX(j+1);
			grid_map[i][j].setY(i+1);
		}
	}

	

}


vector<Path> unconstrainedSearch(const vector< vector<State_map> >& gridmapIn){


}