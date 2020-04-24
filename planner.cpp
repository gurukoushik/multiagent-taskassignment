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

class Node {
private:
    Node* prev_node;
    double cost;
    bool root;
    vector <tuple<int, int, int>> constraints;
    vector <Path> solutions;
    double* assignment;

public:
    Node() {}
    Node(Node* p, bool r) {
        root = r;
        prev_node = p;

    }
    void set_prev_node(Node* p) { prev_node = p; }
    Node* get_prev_node() { return prev_node; }

    void set_cost(double c) { cost = c; }
    double get_cost() { return cost; }

    void set_root(bool r) { root = r; }
    bool get_root() { return root; }

    void push_constraints(int agent, Point index, int time) { constraints.push_back(make_tuple(agent, index, time)); }
    void set_constraints(vector<tuple<int, int, int>> con) { constraints = con; }
    vector<tuple<int, int, int>> get_constraints() { return constraints }

    void set_assignment(double* a) { assignment = a; }
    double* get_assignment() { return assignment }


    void set_solution(vector<Path> s) { solutions = s; }
    vector<Path> get_solution() { return solutions; }
};

struct min_heap {
    bool operator()(Node* p1, Node* p2)
    {
        return p1->get_cost() > p2->get_cost();
    }
};



bool check_conflict(Node* node, int numofagents, tuple<int, int, Point, int> &conflict) {
    vector<Path> sol = node->get_solution();

    for (int i = 0; i < numofagents; i++)
    {
        for (int j = i+1; j < numofagents - 1; j++) {
            int m;

            vector<Point> agent1 = sol[i].pathVect;
            vector<Point> agent2 = sol[j].pathVect;
            m = min((agent1.size(), agent2.size());
            
            for (int k = 0; k < m; k++) {
                if (agent1[k] == agent2[k]) {
                    conflict = make_tuple(i, j, agent1[k], k);

                    return 0;
                }

            }
            
        }

    }
    return 1;
}




double get_SIC(Node* node, int numofagents) {
    double cost = 0;

    vector<Path> solutions = node->get_solution();
    for (int i = 0; i < numofagents; i++)
    {
        cost += solutions[i].cost;

    }
    return cost;
}




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


    

   
    static Node* final_node;
    int goals_reached = 0;

    if (curr_time == 0) {

        //  Defines goal position for Roshan's code
        vector<Point> goals;
        for (int i = 0; i < numofgoals; i++) {
            Point goal(goalpos[i], goalpos[i + numofgoals]); // in terms of x and y positions
            goals.push_back(goal);
        }
        vector<Point> starts;
        for (int i = 0; i < numofagents; i++) {
            Point start(robotpos[i], robotpos[i + numofagents]); // in terms of x and y positions
            starts.push_back(start);
        }



        //  For Guru's part, start and goals are still defined as arrays - double*

        //  Define gridmap for Dijkstra expansions
        State_map state_init_map(numofgoals);
        vector<vector<State_map> > gridmap(y_size, vector<State_map>(x_size, state_init_map));
        backDijkstra(gridmap, goals, map, x_size, y_size);


        priority_queue<Node*, vector<Node*>, min_heap> OPEN;
        int goal_reached = 0;
        Node* start_node = new Node(NULL, 1);

        // call to Guru's initial assignment function. Should return goal positions of type double*. 
        double* goalpos_new = first_assignment(robotpos, goalpos, gridmap);
        start_node->set_assignment(goalpos_new);

        // call to Roshan's low level search with no constraints initially. Should return data structure of type: vector <pair<double, vector<Point>>>
        vector<Point> goals_new;
        for (int i = 0; i < numofgoals; i++) {
            Point goal(goalpos_new[i], goalpos_new[i + numofgoals]); // in terms of x and y positions
            goals_new.push_back(goal);
        }
        start_node->set_solution(unconstrainedSearch(gridmap, starts, goals_new));
        start_node->set_cost(get_SIC(start_node, numofagents));
        OPEN.push(start_node);

        
        while (!OPEN.empty()) {

            Node* curr = OPEN.top();
            OPEN.pop();
            tuple<int, int, Point, int> conflict;


            // no conflicting paths found
            int no_conflict = check_conflict(curr, numofagents, conflict);
            if (no_conflict) {
                goals_reached = 1;
                final_node = curr;
                break;
            }


            // create root node of new tree
            if (curr->get_root()) {
                Node* new_node;
                new_node->set_root(1);

                // In case there are changes in the map like sudden addition of obstacles. Recompute heuristics by calling Roshan's functions. 
                // In this case we'd also need to update the cost of the map.
                // More on this update later
                State_map state_init_map(numofgoals);
                vector<vector<State_map> > gridmap(y_size, vector<State_map>(x_size, state_init_map));
                backDijkstra(gridmap, goals, map, x_size, y_size);

                // call to Guru's new assignment function. Should return goal positions of type double*. Should input updated gridmap if the map changes 
                double* goalpos_new = new_assignment(robotpos, goalpos, gridmap);
                new_node->set_assignment(goalpos_new);

                // call to Roshan's low level search with no constraints initially. Should return data structure of type: vector <pair<double, vector<Point>>>
                vector<Point> goals_new_tree;
                for (int i = 0; i < numofgoals; i++) {
                    Point goal(goalpos_new[i], goalpos_new[i + numofgoals]); // in terms of x and y positions
                    goals_new_tree.push_back(goal);
                }
                new_node->set_solution(unconstrainedSearch(gridmap, starts, goals_new_tree));
                new_node->set_cost(get_SIC(new_node, numofagents));
                OPEN.push(new_node);
            }


            // redefinition of goal for Roshan's code from double* to vector<Point>
            double* goalpos_child = curr->get_assignment();
            vector<Point> goals_child;
            for (int i = 0; i < numofgoals; i++) {
                Point goal(goalpos_child[i], goalpos_child[i + numofgoals]); // in terms of x and y positions
                goals_child.push_back(goal);
            }


            // create child node for conflicting agent 1
            Node* child_node1;
            child_node1->set_constraints(curr->get_constraints());
            child_node1->push_constraints(get<0>(conflict), get<2>(conflict), get<3>(conflict));
            child_node1->set_assignment(goalpos_child);
            child_node1->set_root(0);

            // call to Roshan's low level search with a vector of constraints initially. Should return data structure of type: vector <pair<double, vector<Point>>>
            child_node1->set_solution(constrainedSearch(gridmap, starts, goals_child, child_node1->get_constraints()));
            child_node1->set_cost(get_SIC(child_node1, numofagents));
            OPEN.push(child_node1);


            // create child node for conflicting agent 2
            Node* child_node2;
            child_node2->set_constraints(curr->get_constraints());
            child_node2->push_constraints(get<1>(conflict), get<2>(conflict), get<3>(conflict));
            child_node2->set_assignment(goalpos_child);
            child_node2->set_root(0);

            // call to Roshan's low level search with a vector of constraints. Should return data structure of type: vector <pair<double, vector<Point>>>

            child_node2->set_solution(constrainedSearch(gridmap, starts, goals_child, child_node2->get_constraints()));
            child_node2->set_cost(get_SIC(child_node2, numofagents));
            OPEN.push(child_node2);

        }
    }


    vector<Path> set_of_sol = final_node->get_solution();
    if (goals_reached) {
        for (int i = 0; i < numofagents; i++) {
            vector<Point> sol = set_of_sol[i].pathVect;
            action_ptr[i] = sol[curr_time].x;
            action_ptr[i + numofagents] = sol[curr_time].y;
        }

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