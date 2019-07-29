classdef Source
   properties
       state
   end
   methods
       % methods: return reward from source to a point, true distance from
       % source to agent?
       function obj = Source(x, y, max) % constructor function
          obj.state = [x, y, max];
       end
       function r = Reward(obj, pos)    % return reward from a certain position - 1/r^2
           var = 0.001;
           temp = norm(pos - obj.state(1:end-1));
%            r = -obj.state(end)*4/((1+exp(0.5*temp))*(1+exp(-0.5*temp))) + var*randn();
           r = -obj.state(end)/temp + var*randn();
       end
       function r = returnPos(obj)
          r = obj.state(1:2);
       end
       
   end
end