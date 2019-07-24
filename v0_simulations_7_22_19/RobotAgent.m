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
        function r = returnStates(obj, num)
            max = size(obj.states(1:end, 1), 1);
            if(max > num)
                max = num;
            end
            r = obj.states(end-max+1:end, :);
        end
        
        function r = descDirection(obj, list) % use multi-point gradient algorithm 
            curState = obj.states(end, 1:2);
            curReward = obj.states(end, 3);
            
            % constants relating to history
            beta = 1;
            gamma = 2;
            
            dir = [0, 0];
            % Fix runtime at a certain point
%             max = size(obj.states(1:(end-1)), 1);
%             if(max > 100)
%                 max = 100;
%             end
            for n = 1:size(list, 1)
                checkState = list(n, 1:2);
                checkReward = list(n, 3);

                dif = checkState - curState;
                if dif == 0
                    continue
                end
                temp = (beta/norm(dif)^gamma)*(log(checkReward) - log(curReward)).*(dif/norm(dif));
                
                dir = dir + temp;
            end
            r = dir;
        end
    end
end
    