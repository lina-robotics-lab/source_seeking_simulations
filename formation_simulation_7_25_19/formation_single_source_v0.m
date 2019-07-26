syms x z p;

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

% Calculate COM and rewards
com = [0,0];
origin = robotList(1).returnPos();
y = zeros(1, size(robotList, 2));
C = zeros(size(robotList, 2), 3);

for j = 1:size(robotList, 2)
   temp = robotList(j).returnStates(1);
   com = com + (temp(1:2) - origin);
   y(j) = temp(3);
   
   C(j, 1:end-1) = temp(1:end-1) - com;
   C(j, end) = 1;
end
com = com./size(robotList, 2) + origin;
% disp(C);
% disp(y);


% Calculate Gradient based on COM
grad_est = inv(transpose(C)*C)*transpose(C)*transpose(y);
% disp(grad_est);
% disp(source1.Reward(com));

H(x, z) = [ (8*x^2)/(x^2 + z^2)^3 - 2/(x^2 + z^2)^2, (8*x*z)/(x^2 + z^2)^3;
           (8*x*z)/(x^2 + z^2)^3, (8*z^2)/(x^2 + z^2)^3 - 2/(x^2 + z^2)^2];
               
sigma_h = 0.01;
syms x1 z1 x2 z2 x3 z3;
opt_pos  = @(x1, z1, x2, z2, x3, z3)((1/2)*[x1 z1]*H(com(1), com(2))*[x1; z1])^2*C*inv(transpose(C)*C)*inv(transpose(C)*C)*transpose(C)*sigma_h^2 ...
         +((1/2)*[x2 z2]*H(com(1), com(2))*[x2; z2])^2*C*inv(transpose(C)*C)*inv(transpose(C)*C)*transpose(C)*sigma_h^2 ...
         +((1/2)*[x3 z3]*H(com(1), com(2))*[x3; z3])^2*C*inv(transpose(C)*C)*inv(transpose(C)*C)*transpose(C)*sigma_h^2;
% minim = symsum(opt_pos, x, z);
x0 = [0 0 0; 0 0 0];
for j = 1:size(robotList, 2)
    temp = robotList(j).returnPos();
    x0(1, j) = temp(1);
    x0(2, j) = temp(2);
    disp(temp);
end
disp(x0(:));
final = fminsearch(opt_pos, transpose(x0(:)));
disp(final);

