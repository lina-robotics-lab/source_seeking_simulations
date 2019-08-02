%% Initialize and set up formation potentials

syms f_d(d) f_h(d)
t = 0.1;    % this is s-dot

a_d = 0.5;
d_1 = 4;
d_0 = sqrt(2)*sqrt(3);
f_d = @(d)a_d*(1/d - d_0/d^2);
d_def = feval(f_d, d_1);

a_h = 0.5;
h_1 = 4;
h_0 = sqrt(2);
f_h = @(d)a_h*(1/d - h_0/d^2);
h_def = feval(f_h, h_1);

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

% Virtual leaders
virtLead1 = RobotAgent;
pos = [5,5];
reward = source1.Reward(pos);
virtLead1.addState([pos, reward]);

virtLead2 = RobotAgent;
pos = [0,0.5];
reward = source1.Reward(pos);
virtLead2.addState([pos, reward]);

virtList = [virtLead1];

K_p = 0.1;
vel_max = 0.1;
target = 0.1;
gradients = [];
exit = false;
% Iterative steps
for iter = 1:10000
    if exit
       break 
    end
    %% Robot movement
   for i = 1:size(robotList, 2)
       pos = [0, 0];
       sum = [0 0];
       
       %Calculate robot relations
       for j = 1:size(robotList, 2)
          if i ~= j
              dif = double(robotList(i).returnPos() - robotList(j).returnPos());
              mag = norm(dif);
              if mag > d_1
                 sum = sum - d_def*dif/norm(dif);
              else
                 sum = sum - feval(f_d, mag)*dif/norm(dif);
              end
          end
       end
       
       %Calculate virt leader relations
        for j = 1:size(virtList, 2)
            dif = robotList(i).returnPos() - virtList(j).returnPos();
            mag = norm(dif);
            if mag > h_1
                sum = sum - h_def*dif/norm(dif);
            else
                sum = sum - feval(f_h, mag)*dif/norm(dif);
            end
        end
              
       sum = sum - K_p*(robotList(i).vel);
       
       % Update accel for next move
       robotList(i).acc = sum;
       
       % Move the robot
       pos = double(robotList(i).returnPos() + robotList(i).vel*t + 0.5*robotList(i).acc*t^2);
       reward = source1.Reward(pos);
       robotList(i).addState([pos, reward]);
       
       % Change robot's intial velocity for next cycle
       robotList(i).vel = robotList(i).vel + robotList(i).acc*t;
       if robotList(i).vel > vel_max
           robotList(i).vel = vel_max;
       end
   end
   
   %% Virtual body movement
    
    % Find COM
    com = [0,0];
    origin = virtList(1).returnPos();

    for j = 1:size(virtList, 2)
       temp = virtList(j).returnStates(1);
       com = com + (temp(1:2) - origin);
    end
    com = com/size(virtList, 2) + origin;
    
    y = zeros(1, size(robotList, 2));
    C = zeros(size(robotList, 2), 3);
    for j = 1:size(robotList, 2)
        temp = robotList(j).returnStates(1);
        y(j) = temp(3);

        C(j, 1:end-1) = temp(1:end-1) - com;
        C(j, end) = 1;
    end

    grad_est = transpose(inv(transpose(C)*C)*transpose(C)*transpose(y));
    gradients = cat(1, gradients, grad_est);
   
   % Move the virtual body
    for i = 1:size(virtList, 2)
%         temp = t*grad_est(1:2);
%         if temp > vel_max*0.1
%             pos = virtList(i).returnPos() - vel_max*0.1 * grad_est(1:2)/norm(grad_est(1:2));
%         else
%             pos = virtList(i).returnPos() - t*grad_est(1:2);
%         end        
        pos = virtList(i).returnPos() - t*grad_est(1:2);
        reward = source1.Reward(pos);
        virtList(i).addState([pos, reward]);
    end
    
    % Check for exit
    for j = 1:size(virtList, 2)
        if norm(virtList(j).returnPos() - source1.returnPos()) < target
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
    for j = 1:size(virtList, 2)
        scatter3(virtList(j).states(:, 1), virtList(j).states(:, 2), 1:size(virtList(j).states,1), 10, virtList(j).states(:, 3), 'd');
    end
    hold off
end

disp(iter);