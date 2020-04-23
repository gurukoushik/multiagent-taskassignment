#include "lowlevel.h"

///// class definitions ////////
State_map::State_map(int numofgoalsIn){
	
	for(int i=0; i<numofgoalsIn; i++){

		h_vals.push_back(numeric_limits<double>::infinity());
	}
	expanded = false; goalIndex = -1;
}

void backDijkstra(vector<vector<State_map> >& gridmapIn, const vector<Point>& goals, double*map, 
	int x_size, int y_size, int collision_thresh){

	// populate map
	for (int i =0; i<y_size; i++){
		for (int j=0; j<x_size; j++){

			gridmapIn[i][j].setX(j+1);
			gridmapIn[i][j].setY(i+1);
		}
	}

	int numGoals = goals.size();
	for(int i = 0; i<numGoals; i++){

		// reset bool value
		for (int i1 =0; i1<y_size; i1++){
			for (int j1=0; j1<x_size; j1++){

				gridmapIn[i1][j1].contract();
				gridmapIn[i1][j1].setGoalIndex(i);
			}
		}

		// start state
		gridmapIn[goals[i].y_pos - 1][goals[i].x_pos - 1].setH(0.0);
		priority_queue <State_map, vector<State_map>, CompareF_map> open_set_map;
		open_set_map.push( gridmapIn[goals[i].y_pos - 1][goals[i].x_pos - 1] );

		while(!open_set_map.empty()){

			State_map temp = open_set_map.top();
			int x_temp = temp.getPoint().x_pos;
			int y_temp = temp.getPoint().y_pos;
			double h_temp = temp.getH()[i];
			gridmapIn[y_temp-1][x_temp-1].expand();

			open_set_map.pop();

			// span through succesors
			for(int dir = 0; dir < numOfDirs; dir++){

		        int newx = x_temp + dX[dir];
		        int newy = y_temp + dY[dir];

		        if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size)
		        {
		            if (((int)map[GETMAPINDEX(newx,newy,x_size,y_size)] >= 0) && 
		            ((int)map[GETMAPINDEX(newx,newy,x_size,y_size)] < collision_thresh) && 
		            (!gridmapIn[newy-1][newx-1].isExpanded()) )  //if free
		            {

		            	if( gridmapIn[newy-1][newx-1].getH()[i] > h_temp + 
		            		(int)map[GETMAPINDEX(newx,newy,x_size,y_size)] ){

							gridmapIn[newy-1][newx-1].setH(h_temp + 
								(int)map[GETMAPINDEX(newx,newy,x_size,y_size)]);
							open_set_map.push(gridmapIn[newy-1][newx-1]);
		            	}
		            }
		        }
		    }
		}
	}

	return;
}


vector<Path> unconstrainedSearch(const vector< vector<State_map> >& gridmapIn){


}