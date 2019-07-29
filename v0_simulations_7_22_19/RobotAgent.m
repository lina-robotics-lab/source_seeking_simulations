% Moving sensor agent - holonomic
classdef RobotAgent < handle
    properties
        states = [];
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
            
            % constants relating to history
            beta = 1;   % beta is uesless, not needed
            gamma = 2;
            
            dir = [0, 0];
            for n = 1:size(list, 1)     % Iterate through all states given which is limited
                checkState = list(n, 1:2);
                checkReward = list(n, 3);

                dif = checkState - curState;    % check if they're from the same position, discard b/ divide by 0
                if dif == 0
                    continue
                end
                temp = (beta/norm(dif)^gamma)*((checkReward) - (curReward)).*(dif/norm(dif)); % note log dif in reward
                
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
                
                temp = 0.0000001* ((0.5/norm(dif))^12 - 2*(0.5/norm(dif))^6).*(dif/norm(dif));
                dir = dir + temp;
            end
            r = dir/norm(dir);  % normalize direction vector
        end
    end
end
    