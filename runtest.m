function runtest(problemfile)

[numofagents, numofgoals, mapdims , C, robotstart, goalpose, envmap] = readproblem(problemfile);

close all;

%draw the environment
figure('units','normalized','outerposition',[0 0 1 1]);
imagesc(envmap'); axis square; colorbar; colormap jet; hold on;

%current positions of the target and robot
time = 0;
robotpos = robotstart;
goalpos = goalpose;

numofmoves = zeros(1,numofagents);
caught = false(1,numofgoals);
pathcost = zeros(1,numofagents);

for ii = 1:numofagents
    hr = -1;
    %draw the agent positions
    if (hr ~= -1)
        delete(hr);
    end
    hr = text(robotpos(ii,1), robotpos(ii,2), 'R', 'Color', 'g', 'FontWeight', 'bold');
    %hr = scatter(robotpos(ii,1), robotpos(ii,2), 10, 'g', 'filled');
end
for ii = 1:numofgoals
    ht = -1;
    %draw the goal positions
    if (ht ~= -1)
        delete(ht);
    end
    ht = text(goalpos(ii,1), goalpos(ii,2), 'T', 'Color', 'm', 'FontWeight', 'bold');
    %ht = scatter(goalpos(ii,1), goalpos(ii,2), 10, 'm', 'filled');
end

pause(1.0);

hr_vec = zeros(numofagents);
% robot can take at most as many steps as target takes
while (~all(caught))
    
    % call robot planner to find what they want to do
    newrobotpos = robotplanner(numofagents, numofgoals, mapdims, C, robotpos, goalpos, envmap, time);
    
    if (size(newrobotpos, 1) ~= numofagents  || size(newrobotpos, 2) ~= size(goalpos, 2))
        fprintf(1, 'ERROR: invalid action, check action size in output\n');
        return;
    end
    newrobotpos = cast(newrobotpos, 'like', robotpos);
    if (any(newrobotpos(:,1) < 1) || any(newrobotpos(:,1) > size(envmap, 1)) || ...
            any(newrobotpos(:,2) < 1) || any(newrobotpos(:,2) > size(envmap, 2)))
        fprintf(1, 'ERROR: out-of-map robot position commanded\n');
        return;
    elseif (any(envmap(newrobotpos(:,1), newrobotpos(:,2)) >= C))
        fprintf(1, 'ERROR: planned action leads to collision\n');
        return;
    elseif (any(abs(newrobotpos(:,1)-robotpos(:,1)) > 1) || any(abs(newrobotpos(2)-robotpos(2)) > 1))
        fprintf(1, 'ERROR: invalid action commanded. robot must move on 8-connected grid.\n');
        return;
    end
    
    time = time + 1;
    prev_robotpos = robotpos;
    robotpos = newrobotpos;
        
    for ii = 1:numofagents
        % add cost proportional to time spent planning
        if(caught(ii) ~= true)
            pathcost(ii) = pathcost(ii) + envmap(robotpos(ii,1), robotpos(ii,2));
            numofmoves(ii) = numofmoves(ii) + 1;
        end
        
        hr_vec(ii) = scatter(robotpos(ii,1), robotpos(ii,2), 100, 'g', 'filled');
        h_line = plot([prev_robotpos(ii,1),robotpos(ii,1)],[prev_robotpos(ii,2),robotpos(ii,2)],'w');
        
        % check if target is caught
        thresh = 0.5;
        if (abs(robotpos(ii,1)-goalpos(ii,1)) <= thresh && abs(robotpos(ii,2)-goalpos(ii,2)) <= thresh)
            caught(ii) = true;
        end
    end
    
    pause(0.5);
    
    % delete the positions after every iteration
    if (~all(caught))
        for jj = 1:numofagents
                delete(hr_vec(jj));
        end
    end
end

fprintf(1, '\nRESULT:\n');
fprintf('\tGoals Completed : [');fprintf('%g ', caught);fprintf(']\n');
fprintf('\tTime Taken : %d\n', time);
fprintf('\tMoves Made : [');fprintf('%g ', numofmoves);fprintf(']\n');
fprintf('\tPath Cost : [');fprintf('%g ', pathcost);fprintf(']\n');