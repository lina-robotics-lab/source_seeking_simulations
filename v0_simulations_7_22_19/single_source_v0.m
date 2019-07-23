%Initialize map
% Run loop for certain number of time steps

sourceList = [];
robotList = [];

% Initialize source
source1 = Source(1, 1, 1);
% disp(source1.State);

% Initialize robot, start point, then test several points
robot1 = RobotAgent;
pos = [4,5];
reward = source1.Reward(pos);
robot1.addState([pos, reward]);

for i = 1:4
    dir = [randn(), randn()];
    dir = 0.1 * dir/norm(dir);  % velocity/step size of 0.1

    pos = pos + dir;
    reward = source1.Reward(pos);
    robot1.addState([pos, reward]);
end

% Loop until robot gets close enough source
count = 0;
var = 1;    % variance in step direction
while(norm(pos - [1,1]) > 0.3 && count < 1000)
    dir = robot1.descDirection() + var*randn(1,2);
    dir = 0.1 * dir/norm(dir);
    pos = pos + dir;
    reward = source1.Reward(pos);
    robot1.addState([pos, reward]);
    count = count + 1;
end

figure
scatter3(robot1.states(:, 1), robot1.states(:, 2), 1:size(robot1.states,1), 10, robot1.states(:, 3));