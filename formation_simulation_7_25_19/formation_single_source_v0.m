syms x z p;

% if(nargin < 1)
%     show = true;
% end

% Initialize source
source1 = Source(0,0,4);

% Initialize robots
robot1 = RobotAgent;
pos = [4.9,4.9];
reward = source1.Reward(pos);
robot1.addState([pos, reward]);

robot2 = RobotAgent;
pos = [5.1,5];
reward = source1.Reward(pos);
robot2.addState([pos, reward]);

robot3 = RobotAgent;
pos = [5,5.1];
reward = source1.Reward(pos);
robot3.addState([pos, reward]);

% Compile list of robots
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

H(x, z) = [ (8*x^2)/(x^2 + z^2)^3 - 2/(x^2 + z^2)^2, (8*x*z)/(x^2 + z^2)^3;
               (8*x*z)/(x^2 + z^2)^3, (8*z^2)/(x^2 + z^2)^3 - 2/(x^2 + z^2)^2];
sigma_h = 0.01;
exit = false;
target = 0.3;

gradients = [];
while(~exit && count < 300)
    % Calculate COM and rewards
    com = [0,0];
    origin = robotList(1).returnPos();
    y = zeros(1, size(robotList, 2));
    C = zeros(size(robotList, 2), 3);
    for j = 1:size(robotList, 2)
       temp = robotList(j).returnStates(1);
       com = com + (temp(1:2) - origin);
       y(j) = temp(3);

       C(j, 1:end-1) = temp(1:end-1) - origin;
       C(j, end) = 1;
    end
    com = com/size(robotList, 2) + origin;
%     if norm(com - source1.returnPos()) < target
%             exit = true;
%             break
%     end
    % disp(C);
    % disp(y);


    % Calculate Gradient based on COM
    grad_est = transpose(inv(transpose(C)*C)*transpose(C)*transpose(y));
    gradients = cat(1, gradients, grad_est);
    % disp(grad_est);
    % disp(source1.Reward(com));

    % Calculate Optimal formation positions
    % Build Symbolic minimization eqn
    
    K = C*inv(transpose(C)*C)*inv(transpose(C)*C)*transpose(C);
    opt_pos  = @(xf)((1/2)*[xf(1) xf(2)]*H(com(1), com(2))*[xf(1); xf(2)])^2*K(1,1)*sigma_h^2 ...
             +((1/2)*[xf(3) xf(4)]*H(com(1), com(2))*[xf(3); xf(4)])^2*K(2,2)*sigma_h^2 ...
             +((1/2)*[xf(5) xf(6)]*H(com(1), com(2))*[xf(5); xf(6)])^2*K(3,3)*sigma_h^2 ...
             +trace(K)*0.001;

    % Build initial starting points
    x0 = [0 0 0; 0 0 0];
    for j = 1:size(robotList, 2)
        temp = robotList(j).returnPos();
        x0(1, j) = temp(1) - com(1);
        x0(2, j) = temp(2) - com(2);

    end
    
    % Run built-in minimization algorithm - slow!
    options.MaxFunEvals = 600;
    final = fminsearch(opt_pos, x0(:));
%     final = x0(:);
    x0 = reshape(final, [2, 3]);
    % disp(final);

    % Normalize gradient, move COM and robots ideally
    com_next = com - 0.1*(grad_est(1:2)/norm(grad_est(1:2)));

    for j = 1:size(robotList, 2)
       robot_ideal = com_next + transpose(x0(:, j));
       dir = robotList(j).returnPos() - robot_ideal;
       if norm(dir) < 0.1
           pos = robot_ideal;
       else
           dir = 0.1*dir/norm(dir);
           pos = robotList(j).returnPos() - dir;
       end
       reward = source1.Reward(pos);       % move to position and check reward
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

show = true;
if show
    figure
    stateList = [];
    for j = 1:size(robotList, 2)
        stateList = cat(1, stateList, robotList(j).states);
    end
    scatter3(stateList(:, 1), stateList(:, 2), 1:size(stateList,1), 10, stateList(:, 3));
end

% Return steps before convergence
out = count/size(robotList, 2)