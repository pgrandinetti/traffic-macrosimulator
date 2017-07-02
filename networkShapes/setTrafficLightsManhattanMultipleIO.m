% This file is part of Traffic MacroSimulator.
%
% Traffic MacroSimulator is free software: you can redistribute it and/or
% modify it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% You should have received a copy of the GNU General Public License
% along with Traffic MacroSimulator.  If not, see http://www.gnu.org/licenses/.

% @author: P. Grandinetti

function [] = setTrafficLightsManhattanMultipleIO(n, nCols)

isVert = @(i,nCols) mod(i,2*nCols+1)>0 && mod(i,2*nCols+1)<=nCols;

% set traffic light equal all over the network, giving the following split
% to vertical roads
split = 0.4;

for i = 1:size(n.iM,1)
    list = n.downKin(i);
    if ~isempty(list)
       l1 = floor(n.period*split);
       if isVert(i,nCols)
           n.lights(i).values = [ones(1,l1)  zeros(1,n.period-l1)];
       else
           n.lights(i).values = [zeros(1,n.period-l1) ones(1,l1)];
       end
    else
        n.lights(i).values = ones(1,n.period);
    end
end

end

