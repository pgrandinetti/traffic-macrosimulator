% This file is part of Traffic MacroSimulator.
%
% Traffic MacroSimulator is free software: you can redistribute it and/or
% modify it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% You should have received a copy of the GNU General Public License
% along with Traffic MacroSimulator.  If not, see http://www.gnu.org/licenses/.

% @author: P. Grandinetti

% This class is a modelization of traffic lights:
% In traffic network we define a traffic light as a discrete time signal,
% binary valued, with a property "cycle" and a "duty cycle" (integral
% average over the cycle length"

classdef Light < handle
    
    properties (SetAccess = public)
        id;
        period;
        values;
    end
    
    methods
        
        function L = Light (id, period)
            L.id = id;
            L.period = period;
            %L.values = randn(period-1,1);
        end
        
        function value = valueAt(self, t)
            if (t == 0)
                error ('Trying to access with zero index')
            end
            index = mod(t,self.period);
            if index == 0
                value = self.values(self.period);
            else
                value = self.values(index);
            end
        end
        
        function g = dutyCycle(obj)
            g = sum(obj.values)/obj.period;
        end
        
    end
end

