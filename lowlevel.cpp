#include "lowlevel.h"

///// class definitions ////////
State_map::State_map(int numofgoalsIn){
	
	for(int i=0; i<numofgoalsIn; i++){

		h_val.push_back(numeric_limits<double>::infinity());
	}
	expanded = false;
}

void backDijkstra(vector<vector<State_map> >& gridmapIn, const vector<Point>& goals, int x_size, int y_size){

	// populate map
	for (int i =0; i<y_size; i++){
		for (int j=0; j<x_size; j++){

			gridmapIn[i][j].setX(j+1);
			gridmapIn[i][j].setY(i+1);
		}
	}

	int numGoals = goals.size();
	for(int i = 0; i<numGoals; i++){

		gridmap[goals[i].y_pos - 1][goals[i].x_pos - 1].setH(0.0);
		priority_queue <State_map, vector<State_map>, CompareF_map> open_set_map;
		open_set_map.push( gridmapIn[goals[i].y_pos - 1][goals[i].x_pos - 1] );

		while(!open_set_map.empty()){

			State_map temp = open_set_map.top();
			int x_temp = temp.getPoint().x_pos;
			int y_temp = temp.getPoint().y_pos;
			double h_temp = temp.getH()[i];
			gridmap[y_temp-1][x_temp-1].expand(); // need to initialize this above
		}
	}

}


vector<Path> unconstrainedSearch(const vector< vector<State_map> >& gridmapIn){


}