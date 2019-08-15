classdef RBF
    %RBF Summary of this class goes here
    %   Radial Basis function from Choi, Oh, Horowitz (10)-(11)
    
    properties
        center = [0, 0]
        factor = 1;
        width = 1;
    end
    
    methods
        function obj = RBF(center, factor, width)
            %RBF Construct an instance of this class
            %   center = (1,2); factor = non-zero scalar; width = scalar
            obj.center = center;
            obj.factor = factor;
            obj.width = width;
        end
        
        function r = returnField(obj, pos)
            %returnField Return the field from a basis given a location
            %   pos = (1, 2)
            r = (1/obj.factor)*exp(-1*norm(pos - obj.center)^2/(2 * obj.width^2));
        end
        
        function r = returnGrad(obj, pos)
            x = (1/(obj.factor * obj.width^2))*exp(-1*norm(pos - obj.center)^2/(2 * obj.width^2))*(-obj.center(1) + pos(1));
            y = (1/(obj.factor * obj.width^2))*exp(-1*norm(pos - obj.center)^2/(2 * obj.width^2))*(-obj.center(2) + pos(2));
            r = [x, y];
        end
    end
end

