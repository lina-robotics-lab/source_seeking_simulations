% Moving sensor agent - holonomic
classdef RobotAgent < handle
    properties
        states = [];
        vel = [0, 0];
        acc = [0, 0];
        weights = [];
    end
    methods
        % Constructor
        function obj = RobotAgent(weightlength)
            obj.weights = ones(1, weightlength);
        end
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
        
    end
end
    