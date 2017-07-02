% This file is part of Traffic MacroSimulator.
%
% Traffic MacroSimulator is free software: you can redistribute it and/or
% modify it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% You should have received a copy of the GNU General Public License
% along with Traffic MacroSimulator.  If not, see http://www.gnu.org/licenses/.

% @author: P. Grandinetti

function [] = setTrafficLights(n, nRows, nCols)

isHoriz = @(i) mod(i, nRows+nCols-1)<=nCols-1 & mod(i, nRows+nCols-1)>=1;

% set equal traffic lights (50% to all roads)
for i = 1 : size(n.iM,1)
    list = n.downKin(i);
    if ~isempty(list)
        if isHoriz(i)
            n.lights(i).values = [ones(1,n.period/2) zeros(1,n.period/2)];
        else
            n.lights(i).values = [zeros(1,n.period/2) ones(1,n.period/2)];
        end
    else
        n.lights(i).values = ones(1,n.period);
    end
end

end

