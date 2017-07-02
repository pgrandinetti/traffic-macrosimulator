% This file is part of Traffic MacroSimulator.
%
% Traffic MacroSimulator is free software: you can redistribute it and/or
% modify it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% You should have received a copy of the GNU General Public License
% along with Traffic MacroSimulator.  If not, see http://www.gnu.org/licenses/.

% @author: P. Grandinetti

function Din = inputDemandManhattan(net, burstValue, t0, tf, Tsim)

Din = zeros(length(net.roads),Tsim);

for i=1:size(Din,1)
    if isempty(net.neighborsIn(i))
        Din(i,t0:tf) = burstValue;
    end
end

end

