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

	printf("map populated\n");

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
		gridmapIn[goals[i].y_pos - 1][goals[i].x_pos - 1].setH(i, 0.0);
		printf("h value at goal %d is %lf\n", i, gridmapIn[goals[i].y_pos - 1][goals[i].x_pos - 1].getH()[i]);
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
		        // printf("evaluated newx = %d, newy = %d\n", newx, newy);

		        if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size)
		        {
		            if (((int)map[GETMAPINDEX(newx,newy,x_size,y_size)] >= 0) && 
		            ((int)map[GETMAPINDEX(newx,newy,x_size,y_size)] < collision_thresh) && 
		            (!gridmapIn[newy-1][newx-1].isExpanded()) )  //if free
		            {

		            	printf("gridmapIn LHS = %lf, RHS = %lf\n", gridmapIn[newy-1][newx-1].getH()[i], 
		            		h_temp + (int)map[GETMAPINDEX(newx,newy,x_size,y_size)]);
		            	if( gridmapIn[newy-1][newx-1].getH()[i] > h_temp + 
		            		(int)map[GETMAPINDEX(newx,newy,x_size,y_size)] ){

							gridmapIn[newy-1][newx-1].setH(i, h_temp + 
								(int)map[GETMAPINDEX(newx,newy,x_size,y_size)]);
							open_set_map.push(gridmapIn[newy-1][newx-1]);
							printf("Pushed to open_set_map \n");
		            	}
		            }
		        }
		    }
		}		
	}

	return;
}


vector<Path> unconstrainedSearch(const vector< vector<State_map> >& gridmapIn, const vector<Point>& robotPosnsIn, 
	const vector<int>& assignment, const vector<Point>& goalsIn, int x_size, int y_size){

	vector<Path> outPaths;	

	for(int i=0; i<robotPosnsIn.size(); i++){

		int goalIdx = assignment[i];
		if(goalIdx>=goalsIn.size()){
			printf("goalidx is greater than no of goals\n");
			continue;
		}

		stack<State_map> optPath;
		Path tempPath;

		printf("\n before pushing to optPath \n");

		optPath.push( gridmapIn[robotPosnsIn[i].y_pos-1][robotPosnsIn[i].x_pos-1] );
		tempPath.pathVect.push_back(gridmapIn[robotPosnsIn[i].y_pos-1][robotPosnsIn[i].x_pos-1].getPoint());

		printf("\n before entering while loop \n");

		while( !optPath.empty() && !(optPath.top().getPoint() == gridmapIn[goalsIn[goalIdx].y_pos-1][goalsIn[goalIdx].x_pos-1].getPoint()) ){

			double min_H = numeric_limits<double>::infinity(); 
			int finX, finY;

			for(int dir1 = 0; dir1 < numOfDirs; dir1++){

		        int newx = optPath.top().getPoint().x_pos + dX[dir1];
		        int newy = optPath.top().getPoint().y_pos + dY[dir1];

		        if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size && min_H > gridmapIn[newy-1][newx-1].getH()[goalIdx]){

					min_H = gridmapIn[newy-1][newx-1].getH()[goalIdx];
					finX = newx; finY = newy;
		        }
		    }
		   
		    optPath.push(gridmapIn[finY-1][finX-1]);
		    tempPath.pathVect.push_back(gridmapIn[finY-1][finX-1].getPoint());		    
		}
		printf("cost of path that goes from robot %d to goal %d is ", i, goalIdx);
		printf("%lf\n", gridmapIn[robotPosnsIn[i].y_pos-1][robotPosnsIn[i].x_pos-1].getH()[goalIdx]);
		tempPath.cost = gridmapIn[robotPosnsIn[i].y_pos-1][robotPosnsIn[i].x_pos-1].getH()[goalIdx];
		outPaths.push_back(tempPath);
	}

	return outPaths;
}


vector<Path> constrainedSearch(const vector< vector<State_map> >& gridmapIn, const vector<Point>& robotPosnsIn, 
	const vector<int>& assignment, const vector<Point>& goalsIn, const vector<tuple<int, int, int>>& tempConstr, 
        int x_size, int y_size){

	int numofagents = robotPosnsIn.size();
	for(int i=0; i < numofagents; i++){

		// initialize stuff
		int curr_time=0;
		robotposeX = robotPosnsIn[i].x_pos; robotposeY = robotPosnsIn[i].y_pos;
		unsigned long long index_temp = 0; double g_temp =0; 
		int x_temp = robotposeX; int y_temp = robotposeY; int t_temp = curr_time; 

		int goalIdx = assignment[i];
		if(goalIdx>=goalsIn.size()){

			printf("goalidx is greater than no of goals\n");
			continue;
		}

		// set up hash table
	    unordered_set<unsigned long long, double> closed_set;

	    //robot start state
	    Node_time* rob_start = new Node_time;
	    rob_start->setX(robotposeX); rob_start->setY(robotposeY); rob_start->setT(curr_time);
	    rob_start->setG(0.0);
	    rob_start->setH( gridmapIn[robotposeY - 1][robotposeX - 1].getH()[goalIdx] );

	    priority_queue <Node_time*, vector<Node_time*>, CompareF_time> open_set;

	    // start while loop for A* expansion
		while( !open_set.empty() ){

			Node_time* tempPtr = open_set.top();			

			int x_temp = tempPtr->getX(), y_temp = tempPtr->getY(), t_temp = tempPtr->getT();
			double g_temp = tempPtr->getG();
			tempPtr->expand();
			closed_set.insert(GetIndex(tempPtr));
			open_set.pop();

			// span through successors at next time step
			t_ct++;

			for(int dir = 0; dir < numOfDirs; dir++){

		        int newx = x_temp + dX[dir], newy = y_temp + dY[dir];
		        int newt = t_temp + 1;
		        unsigned long long index_temp = GetIndex(tempPtr);

		        if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size && 
		        	((int)map[GETMAPINDEX(newx,newy,x_size,y_size)] >= 0) &&
		        	((int)map[GETMAPINDEX(newx,newy,x_size,y_size)] < collision_thresh) && 
		        	(closed_set.find(index_temp)==closed_set.end()) &&
		        	 CBSOkay(ConstrIn, newx, newy, newt, i) ){

					Node_time* newNode =  new Node_time;
					newNode->setX(newx); newNode->setY(newy); newNode->setT(newt);
					newNode->setG(g_temp + (int)map[GETMAPINDEX(newx,newy,x_size,y_size)]);
					newNode->setH( gridmapIn[newy - 1][newx - 1].getH()[goalIdx] );
					
					newNode->setParent(tempPtr);
					tempPtr->addSuccessor(newNode);
					open_set.push(newNode);
		        }
		    }

		    // if (t_ct >= 790000){cout << "Unable to find a solution\n"<<endl;}
		}
	}

}




// helper functions
unsigned long long GetIndex(Node_time* tempPtrIn){

	int x = tempPtrIn->getX(); int y = tempPtrIn->getY(); int t = tempPtr->getT();

	// return t*(2*t-1)*(2*t+1)/3 + (y+t)*(2*t+1) + x + t

	// return ( (double) ((x + y) * (x + y + 1.0)/2.0 + y + t)*((x + y) 
	// 	* (double)(x + y + 1.0)/2.0 + y + t + 1.0) )/2.0 + t;

	return floor( 1000000* (sqrt(2)*x + sqrt(5)*y + sqrt(11)*t) );
}

// CBS constraint
bool CBSOkay(const vector<tuple<int, int, int>>& tempConstr, int newx, int newy, int newt, int i){

	if(i == )
}