function [action,assign] = robotplanner(numofagents, numofgoals, map_dim, collision_thresh, robotpose, goalpose, map, curr_time)

[action,assign] = planner(numofagents, numofgoals, map_dim, collision_thresh, robotpose, goalpose, map, curr_time);

end