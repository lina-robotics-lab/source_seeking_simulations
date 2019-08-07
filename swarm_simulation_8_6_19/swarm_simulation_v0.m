clear all
%% Set up surveillance region

M = [-5, 5;
     -5, 5];

RBF_list = [];
for n = 1:5
    for m = 1:5
        temp = RBF([(-6 + n*2), (-6 + m*2)], 1, 1);
        RBF_list = cat(1, RBF_list, temp);
    end
end

test = [0,0];
sum = 0;
for i = 1:size(RBF_list, 2)
    sum = sum + RBF_list(i).returnField(test);
end
%% Set up leaders and agents
source1 = Source(0, 0, 1);
% Robots
robot1 = RobotAgent(size(RBF_list, 2));
% robot1.weights = zeros(1,25);
pos = [5,5];
reward = source1.Reward(pos);
robot1.addState([pos, reward]);

robot2 = RobotAgent(size(RBF_list, 2));
pos = [5, 4.9];
reward = source1.Reward(pos);
robot2.addState([pos, reward]);

robot3 = RobotAgent(size(RBF_list, 2));
pos = [5, 4];
reward = source1.Reward(pos);
robot3.addState([pos, reward]);

robotList = [robot1];


s = 10;
%% Initial Move
for bot = robotList
    % Collect states/rewards
    states = [];
    for i_bot = robotList
        states = cat(1, states, i_bot.returnStates(s));
    end
    Y = states(:, 3);

    % Phi calculation
    cap_phi = [];
    for i = 1:size(states, 1)
        phi = [];
        for rbf = transpose(RBF_list)
            phi = cat(1, phi, rbf.returnField(states(i, 1:2)));
        end
        cap_phi = cat(2, cap_phi, phi);
    end
    cap_phi = transpose(cap_phi);

    prev_P = inv(transpose(cap_phi)*cap_phi);
    prev_cap_theta = prev_P*transpose(cap_phi)*Y;

    grad = [];
    for rbf = transpose(RBF_list)
        grad = cat(1, grad, rbf.returnGrad(states(i, 1:2)));
    end
    grad_mu = transpose(grad)*prev_cap_theta;
    pos = bot.returnPos() + [0.05, 0];
    reward = source1.Reward(pos);
    bot.addState([pos, reward]);

end

for iter = 1:10
    
    %% Calculate P and Y - single robot case for now
    % Iterate through every agent
    for bot = robotList
        % Collect states/rewards
        states = [];
        for i_bot = robotList
            states = cat(1, states, i_bot.returnStates(s));
        end
        Y = states(:, 3);
        
        % Phi calculation
        cap_phi = [];
        for i = 1:size(states, 1)
            phi = [];
            for rbf = transpose(RBF_list)
                phi = cat(1, phi, rbf.returnField(states(i, 1:2)));
            end
            cap_phi = cat(2, cap_phi, phi);
        end
        cap_phi = transpose(cap_phi);
        K = prev_P*transpose(cap_phi)*inv(eye(size(cap_phi, 1)) + cap_phi * prev_P * transpose(cap_phi));
        prev_P = (eye(size(prev_P, 1)) - K * cap_phi) * prev_P;
        prev_cap_theta = prev_cap_theta + K * (Y - cap_phi * prev_cap_theta);
        
%         P = inv(transpose(cap_phi)*cap_phi);
%         cap_theta = P*transpose(cap_phi)*Y;
%         
        grad = [];
        for rbf = transpose(RBF_list)
            grad = cat(1, grad, rbf.returnGrad(states(i, 1:2)));
        end
        grad_mu = transpose(grad)*prev_cap_theta;
        
    end
    
    
end    