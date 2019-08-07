clear all
t = 0.1;

%% Set up leaders and agents
source1 = Source(0, 0, 1);
% Robots
robot1 = RobotAgent;
pos = [6,5];
reward = source1.Reward(pos);
robot1.addState([pos, reward]);

robot2 = RobotAgent;
pos = [5, 6];
reward = source1.Reward(pos);
robot2.addState([pos, reward]);

robot3 = RobotAgent;
pos = [5, 4];
reward = source1.Reward(pos);
robot3.addState([pos, reward]);

robotList = [robot1 robot2 robot3];

%% Iteration
K_p = 0.1;
var = 0.0;
vel_max = 0.1;
target = 0.1;
exit = false;
for iter = 1:1000
    if exit
       break 
    end
    
    %% Move robots
    % Get state information
    stateList = [];
    curPosList = [];
    for j = 1:size(robotList, 2)
        temp = robotList(j).returnStates(100);
        stateList = cat(1, stateList, robotList(j).returnStates(100));
        curPosList = cat(1, curPosList, robotList(j).returnPos());
        
%         R = corrcoef(temp(:,1),temp(:,2));
%         var = R(1,2)*0.3;
    end
    
    for j = 1:size(robotList, 2)
       dir = robotList(j).descDirection(stateList, curPosList);%var*randn(1,2); 
       robotList(j).acc = dir;
       
        % Move the robot
        pos = double(robotList(j).returnPos() + robotList(j).vel*t + 0.5*robotList(j).acc*t^2);
%         if norm(robotList(j).vel) == vel_max
%             pos = double(robotList(j).returnPos() + robotList(j).vel*t);
%         else
%             pos = double(robotList(j).returnPos() + robotList(j).vel*t + 0.5*robotList(j).acc*t^2);
%         end
        reward = source1.Reward(pos);
        robotList(j).addState([pos, reward]);
       
       % Change robot's intial velocity for next cycle
       robotList(j).vel = robotList(j).vel + robotList(j).acc*t;
       if norm(robotList(j).vel) > vel_max
           robotList(j).vel = vel_max * robotList(j).vel/norm(robotList(j).vel);
       end
    end
    
    for j = 1:size(robotList, 2)
        if norm(robotList(j).returnPos() - source1.returnPos()) < target
            exit = true;
            break
        end
    end
end

%% Plotting
show = true;
if show
    figure
    hold on
    grid on
    rotate3d on
    for j = 1:size(robotList, 2)
        scatter3(robotList(j).states(:, 1), robotList(j).states(:, 2), 1:size(robotList(j).states,1), 10, robotList(j).states(:, 3));
    end
%     for j = 1:size(virtList, 2)
%         scatter3(virtList(j).states(:, 1), virtList(j).states(:, 2), 1:size(virtList(j).states,1), 10, virtList(j).states(:, 3), 'd');
%     end
    hold off
end

disp(iter);