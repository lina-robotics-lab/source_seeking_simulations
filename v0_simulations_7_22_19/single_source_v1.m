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

robot4 = RobotAgent;
pos = [5, 5];
reward = source1.Reward(pos);
robot4.addState([pos, reward]);


robotList = [robot1, robot2, robot3, robot4];
% for bot = robotList
%     bot.vel = [0, 0.1*randn()];
% end
% robot1.acc = [0.1, 0];

% pos = double(robotList(1).returnPos() + robotList(1).vel*t + 0.5*robotList(1).acc*t^2);

%% Iteration
K_p = 0.1;
var = 0.0;
vel_max = 0.1;
target = 0.3;
exit = false;
for iter = 1:1000*2
    if exit
       break 
    end
    
    %% Move robots
    % Get state information
    stateList = [];
    curPosList = [];
    for j = 1:size(robotList, 2)
        temp = robotList(j).returnStates(100);
        stateList = cat(1, stateList, robotList(j).returnStates(200));
        curPosList = cat(1, curPosList, robotList(j).returnPos());
        
        if iter > 1
            R = corrcoef(temp(:,1),temp(:,2));
            var = R(1,2)^2*0.05;
            if isnan(var)
                var = 0;
            end
        end
    end
    
    for j = 1:size(robotList, 2)
       dir = robotList(j).descDirection(stateList, curPosList) + var*randn(1,2); 
       if isnan(dir)
           dir = [0, 0];
       end
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

gif = false;
if gif
    stateList = [];
    for k = 1:length(robotList(1).states(:, 1))
       for j = 1:size(robotList, 2)
           stateList = cat(1, stateList, robotList(j).states(k, :));
       end
    end

    figure
    axes('position',[0 0 1 1]);
    plot1 = scatter3(stateList(1, 1),stateList(1, 2),1, 10,stateList(1, 3));
    view(20, 45);
    xlim([-1 7]);
    ylim([-3 7]);
    zlim([-1, iter*4]);
    gif('single_source_v1.gif', 'DelayTime',0.001);
    for k = 2:length(stateList(:, 1))
        plot1.XData = stateList(1:k, 1);
        plot1.YData = stateList(1:k, 2);
        plot1.ZData = 1:k;
        plot1.CData = stateList(1:k, 3);
        gif;
    %     pause(0.001);
    end
end
disp(iter);