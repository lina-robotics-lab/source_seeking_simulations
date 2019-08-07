% Moving sensor agent - holonomic
classdef RobotAgent < handle
    properties
        states = [];
        vel = [0, 0];
        acc = [0, 0];
    end
    methods
        %Methods needed: calculate the direction of descent, move to the
        %next point, append to reward list
        function r = addState(obj, state)   % append state to state list aka next point
            temp = [obj.states; state];
            obj.states = temp;
        end
        function r = returnPos(obj)   % return latest position
            r = obj.states(end, 1:2);
        end
        function r = returnStates(obj, num) % return latest [num] of positions
            max = size(obj.states(1:end, 1), 1);
            if(max > num)
                max = num;
            end
            r = obj.states(end-max+1:end, :);
        end
        
        function r = descDirection(obj, list, curpos) % use multi-point gradient algorithm 
            curState = obj.states(end, 1:2);
            curReward = obj.states(end, 3);
            max = 0;
            
            % constants relating to history
            beta = 1/(2*pi);   % beta is uesless, not needed in the basic formulation
            gamma = 2;
            
            dir = [0, 0];
            for n = 1:size(list, 1)     % Iterate through all states given which is limited
                checkState = list(n, 1:2);
                checkReward = list(n, 3);

                dif = checkState - curState;    % check if they're from the same position, discard b/ divide by 0
                if dif == 0
                    continue
                end
%                 temp = (beta/norm(dif)^gamma)*((checkReward) - (curReward)).*(dif/norm(dif)); 
                temp = (beta/(2^norm(dif)))*((checkReward) - (curReward)).*(dif/norm(dif)); 
%                 temp = (exp(-0.5*norm(dif)^2))*((checkReward) - (curReward)).*(dif/norm(dif));   %Nesterov and Spoikony
                if norm(temp) > max
                    max = norm(temp);
                end

                dir = dir + temp;
            end
            
            alpha = 0.3;
            delta = 0.4;
            for n = 1:size(curpos, 1)
                checkState = curpos(n, 1:2);

                dif = checkState - curState;    % check if they're from the same position, discard b/ divide by 0
                if dif == 0
                    continue
                end
                
%                 temp = -curReward* ((0.5/norm(dif))^6 - 2*(0.5/norm(dif))^4).*(dif/norm(dif));
                temp = -0.1/(norm(dif)^gamma).*(dif/norm(dif));
                dir = dir + temp;
            end
            r = max*dir/norm(dir);  % normalize direction vector
        end
    end
end
    