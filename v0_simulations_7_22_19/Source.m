classdef Source
   properties
       state
   end
   methods
       % methods: return reward from source to a point, true distance from
       % source to agent?
       function obj = Source(x, y, max)
          obj.state = [x, y, max];
       end
       function r = Reward(obj, pos)
           temp = (pos - obj.state(1:end-1)).^2;
           r = obj.state(end)/sum(temp);
       end
       
   end
end