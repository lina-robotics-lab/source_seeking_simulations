% Initialize source
source1 = Source(0,0,1);

% Initialize robots
robot1 = RobotAgent;
pos = [5,5];
reward = source1.Reward(pos);
robot1.addState([pos, reward]);

robot2 = RobotAgent;
pos = [5,5];
reward = source1.Reward(pos);
robot2.addState([pos, reward]);

robot3 = RobotAgent;
pos = [5,5];
reward = source1.Reward(pos);
robot3.addState([pos, reward]);

% Compile list of robots
robotList = [robot1, robot2, robot3];

% Establish initial test points
count = 0;
for i = 1:4
    for j = 1:size(robotList, 2)
        dir = [randn(), randn()];
        dir = 0.1 * dir/norm(dir);  % velocity/step size of 0.1

        pos = robotList(j).returnPos() + dir;
        reward = source1.Reward(pos);       % move to position and check reward
        robotList(j).addState([pos, reward]);
%         count = count + 1;
    end
end

% Calculate COM
com = [0,0];
origin = robotList(1).returnPos();
for j = 1:size(robotList, 2)
   com = com + (robotList(j).returnPos() - origin);
end
com = com./size(robotList, 2);

% Calculate 

disp(com);
