function [action] = robotplanner(numofagents, numofgoals, map_dim, collision_thresh, robotpose, goalpose, map, curr_time)

action = planner(numofagents, numofgoals, map_dim, collision_thresh, robotpose, goalpose, map, curr_time);

end