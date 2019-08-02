syms f_d(d) f_h(d)
t = 0.1;

a_d = 1;
d_0 = sqrt(2)*sqrt(3);
f_d(d) = a_d*(1/d - d_0/d^2);
d_def = f_d(4);

a_h = 4;
h_0 = sqrt(2);
f_h(d) = a_h*(1/d - h_0/d^2);
h_def = f_h(4);

source1 = Source(0, 0, 1);
% Robots
robot1 = RobotAgent;
pos = [1,1];
reward = source1.Reward(pos);
robot1.addState([pos, reward]);

robot2 = RobotAgent;
pos = [-1, 1];
reward = source1.Reward(pos);
robot2.addState([pos, reward]);

robot3 = RobotAgent;
pos = [-1, -1];
reward = source1.Reward(pos);
robot3.addState([pos, reward]);

robotList = [robot1 robot2 robot3];

% Virtual leaders
virtLead = RobotAgent;
pos = [0,0];
reward = source1.Reward(pos);
virtLead.addState([pos, reward]);

virtList = [virtLead];

K_p = 0.1;
% Iterative steps
for iter = 1:1000
   for i = 1:size(robotList, 2)
       pos = [0, 0];
       sum = [0 0];
       
       
       %Calculate robot relations
       for j = 1:size(robotList, 2)
          if i ~= j
              dif = double(robotList(i).returnPos() - robotList(j).returnPos());
              mag = norm(dif);
              if mag > 4
                 mag = 4; 
              end
              sum = sum - double(f_d(mag))*dif/norm(dif);
          end
       end
       
       %Calculate virt leader relations
       for j = 1:size(virtList, 2)
           dif = double(robotList(i).returnPos() - virtList(j).returnPos());
           mag = norm(dif);
          if mag > 4
             mag = 4; 
          end
           sum = sum - double(f_h(mag))*dif/norm(dif);
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
       
   end
end

show = true;
if show
    figure
    stateList = [];
    for j = 1:size(robotList, 2)
        stateList = cat(1, stateList, robotList(j).states);
    end
    scatter3(stateList(:, 1), stateList(:, 2), 1:size(stateList,1), 10, stateList(:, 3));
end