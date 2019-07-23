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
%         function r = addReward(obj, reward)
%             temp = [obj.states; reward];
%             obj.rewards = temp;            
%         end
        function r = returnState(obj)   % return latest state
            r = obj.states(end, :);
        end
%         function r = returnReward(obj)
%             r = obj.states(end, :);
%         end
        
        function r = descDirection(obj) % use multi-point gradient algorithm 
            curState = obj.states(end, 1:2);
            curReward = obj.states(end, 3);
            
            beta = 1;
            gamma = 2;
            
            dir = [0, 0];
            max = size(obj.states(1:(end-1)), 1);
            if(max > 100)
                max = 100;
            end
            for n = max
                checkState = obj.states(end-n, 1:2);
                checkReward = obj.states(end-n, 3);
%                 if(checkState == curState)
%                     continue;
%                 end
                dif = checkState - curState;
                temp = (beta/norm(dif)^gamma)*(log(checkReward) - log(curReward)).*(dif/norm(dif));
                
                dir = dir + temp;
            end
            r = dir;
        end
    end
end
    