/*=================================================================
 *
 * planner.cpp
 *
 *=================================================================*/

/* Include Required Header Files */
#include <math.h>
#include <mex.h>
#include <algorithm>
#include <limits>
#include <vector>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include "lowlevel.h"
#include "param.h"

using namespace std;

/* Input Arguments */
#define	NUM_OF_AGENTS           prhs[0]
#define	NUM_OF_GOALS            prhs[1]
#define	MAP_DIM                 prhs[2]
#define	COLLISION_THRESH        prhs[3]
#define	ROBOT_POS               prhs[4]
#define	GOAL_POS                prhs[5]
#define MAP_IN                  prhs[6]
#define	CURR_TIME               prhs[7]

/* Output Arguments */
#define	ACTION_OUT              plhs[0]

/* access to the map is shifted to account for 0-based indexing in the map,
whereas 1-based indexing in matlab (so, robotpose and goalpose are 1-indexed) */
#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

/* Define MAX and MIN as a function */
#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

/* Number of directions possible to move along */
// #define numOfDirs 9

// 9-Connected Grid
// int dX[numOfDirs] = {-1, -1, -1,  0,  0,  1, 1, 1, 0};
// int dY[numOfDirs] = {-1,  0,  1, -1,  1, -1, 0, 1, 0};

static void planner(
        int numofagents,
        int numofgoals,
        int x_size,
        int y_size,
        int collision_thresh,
        double* robotpos,
        double* goalpos,
        double*	map,
        int curr_time,
        double* action_ptr
        )
{   
    //Define starts and goals (num of goals >= num of robots)
    vector<Point> goals;
    for(int i=0; i<numofgoals; i++){

        Point goal((int) goalpos[i], (int) goalpos[i+numofgoals]); // in terms of x and y positions
        // Guru pls check ^
        goals.push_back(goal);
    }
    vector<Point> robotPosns;
    for(int i=0; i<numofagents; i++){

        Point robPosn((int) robotpos[i], (int) robotpos[i+numofagents]); // in terms of x and y positions
        // Guru pls check ^
        robotPosns.push_back(robPosn);
    }

    //Define gridmap for Dijkstra expansions
    State_map state_init_map(numofgoals);
    vector<vector<State_map> > gridmap(y_size, vector<State_map>(x_size, state_init_map));
    backDijkstra(gridmap, goals, map, x_size, y_size, collision_thresh);
    printf("Dijkstra done\n");

    // define dummy assignmentVect for testing, overwrite when actual assignmentVect is obtained
    vector<int> assignmentVect;
    for(int i=0; i<numofagents;i++){

        assignmentVect.push_back(numofgoals - 1 - i);
    }


    // low-level search (call wherever you want, need to have an assignmentVect first)
    vector<Path> lowLevelPaths = unconstrainedSearch(gridmap, robotPosns, assignmentVect, goals, x_size, y_size);
    printf("unconstrainedSearch done \n");

    // Rest of the stuff goes here
    // Below is what the constrained search should look like
    vector<tuple<int, Point, int>> tempConstr;
    vector<Path> lowLevelPathsConst = constrainedSearch(gridmap, robotPosns, assignmentVect, goals, tempConstr, 
        x_size, y_size, map, collision_thresh);


    for(int i=0; i<numofagents; i++)
    {
        action_ptr[i] = robotpos[i] - 1;
        action_ptr[i+numofagents] = robotpos[i+numofagents] + 0;
        // mexPrintf("%f %f\n",robotpos[i],robotpos[i+numofagents]);
    }
}

// prhs contains input parameters (4):
// 1st is matrix with all the obstacles
// 2nd is a row vector <x,y> for the robot position
// 3rd is a matrix with the target trajectory
// 4th is an integer C, the collision threshold for the map
// plhs should contain output parameters (1):
// 1st is a row vector <dx,dy> which corresponds to the action that the robot should make
void mexFunction( int nlhs, mxArray *plhs[],
        int nrhs, const mxArray*prhs[] )
{
    /* Check for proper number of arguments */
    if (nrhs != 8) {
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
                "Eight input arguments required.");
    } else if (nlhs != 1) {
        mexErrMsgIdAndTxt( "MATLAB:planner:maxlhs",
                "One output argument required.");
    }
    
    int numofagents = (int) mxGetScalar(NUM_OF_AGENTS);
    int numofgoals = (int) mxGetScalar(NUM_OF_GOALS);
    
    /* get the dimensions of the map*/
    int dim_M = mxGetM(MAP_DIM);
    int dim_N = mxGetN(MAP_DIM);
    if(dim_M != 1 || dim_N != 2)
    {
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidmapdimensions",
                "mapdimensions vector should be 1 by 2.");
    }
    double* mapdims = mxGetPr(MAP_DIM);
    int x_size = (int)mapdims[0];
    int y_size = (int)mapdims[1];
    
    /* get the dimensions of the map and the map matrix itself*/
    double* map = mxGetPr(MAP_IN);
    
    /* Get collision threshold for problem */
    int collision_thresh = (int) mxGetScalar(COLLISION_THRESH);
    
    /* get the current timestep the target is at*/
    int curr_time = mxGetScalar(CURR_TIME);
    
    /* get the current robot pose*/
    int robotpose_M = mxGetM(ROBOT_POS);
    int robotpose_N = mxGetN(ROBOT_POS);
    
    if(robotpose_M != numofagents || robotpose_N != 2)
    {
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidrobotpose",
                "robotpose vector should be NUMOFAGENTS by 2 matrix.");
    }
    double* robotpose = mxGetPr(ROBOT_POS);
    
    /* get the goal pose*/
    int goalpose_M = mxGetM(GOAL_POS);
    int goalpose_N = mxGetN(GOAL_POS);
    
    if(goalpose_M != numofgoals || goalpose_N != 2)
    {
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidgoalpose",
                "goalpose vector should be NUMOFGOALS by 2 matrix.");
    }
    double* goalpose = mxGetPr(GOAL_POS);
    
    /* Create a matrix for the return action */ 
    ACTION_OUT = mxCreateNumericMatrix( (mwSize)numofagents, (mwSize)2, mxDOUBLE_CLASS, mxREAL); 
    double* action_ptr = (double*) mxGetData(ACTION_OUT);
    
    /* Do the actual planning in a subroutine */
    planner(numofagents, numofgoals, x_size, y_size, collision_thresh, robotpose, goalpose, map, curr_time, &action_ptr[0]);
    return;
}