%Initialize map
% Run loop for certain number of time steps
function y = single_source_v0(show)
sourceList = [];
if(nargin < 1)
    show = true;
end


% Initialize source
source1 = Source(0, 0, 1);
% disp(source1.State);

% Initialize robot, start point, then test several points
robot1 = RobotAgent;
pos = [5,5];
reward = source1.Reward(pos);
robot1.addState([pos, reward]);

robot2 = RobotAgent;
pos = [5,5.1];
reward = source1.Reward(pos);
robot2.addState([pos, reward]);

robot3 = RobotAgent;
pos = [5.1,5];
reward = source1.Reward(pos);
robot3.addState([pos, reward]);

% Test between 1 and multiple robot cases
% robotList = [robot1];
% robotList = [robot1, robot2];
robotList = [robot1, robot2, robot3];

% Establish initial test points
count = 0;
% for i = 1:4
%     for j = 1:size(robotList, 2)
%         dir = [randn(), randn()];
%         dir = 0.1 * dir/norm(dir);  % velocity/step size of 0.1
% 
%         pos = robotList(j).returnPos() + dir;
%         reward = source1.Reward(pos);       % move to position and check reward
%         robotList(j).addState([pos, reward]);
% %         count = count + 1;
%     end
% end

% Loop until robot gets close enough source
exit = false;
var = 0.1;    % variance in step direction
target = 0.3;
while(~exit && count < 1000)
    stateList = [];
    
    % Get all state information from all robots
    for j = 1:size(robotList, 2)
        stateList = cat(1, stateList, robotList(j).returnStates(10));
    end
    
    % For all robots find direction and move
    for j = 1:size(robotList, 2)
        dir = robotList(j).descDirection(stateList) + var*randn(1,2);
%         dir = robotList(j).descDirection(robotList(j).returnStates(10)) + var*randn(1,2); % Find descent direction, vary it
        dir = 0.1 * dir/norm(dir);
        pos = robotList(j).returnPos() + dir;
        reward = source1.Reward(pos);
        robotList(j).addState([pos, reward]);
        count = count + 1;
    end
    
    % Check if any robots are close enough to target
    for j = 1:size(robotList, 2)
        if norm(robotList(j).returnPos() - source1.returnPos()) < target
            exit = true;
            break
        end
    end
end

% Plot figures
if show
    figure
    stateList = [];
    for j = 1:size(robotList, 2)
        stateList = cat(1, stateList, robotList(j).states);
    end
    scatter3(stateList(:, 1), stateList(:, 2), 1:size(stateList,1), 10, stateList(:, 3));
end

% Return steps before convergence
y = count/size(robotList, 2);
end

% scatter3(robot1.states(:, 1), robot1.states(:, 2), 1:size(robot1.states,1), 10, robot1.states(:, 3));
% scatter3(robot2.states(:, 1), robot2.states(:, 2), 1:size(robot2.states,1), 10, robot2.states(:, 3));