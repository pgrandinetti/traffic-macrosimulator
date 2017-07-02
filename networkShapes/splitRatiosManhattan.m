% This file is part of Traffic MacroSimulator.
%
% Traffic MacroSimulator is free software: you can redistribute it and/or
% modify it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% You should have received a copy of the GNU General Public License
% along with Traffic MacroSimulator.  If not, see http://www.gnu.org/licenses/.

% @author: P. Grandinetti

function [] = splitRatiosManhattan(net, nRows, nCols)

n.turnings = zeros(size(net.iM,1),size(net.iM,1));

isHoriz = @(i) mod(i, nRows+nCols-1)<=nCols-1 & mod(i, nRows+nCols-1)>=1;

% build the split ratio matrix
% preference given to continue in the current direction
% or leaving the network when next to the boundary
split = 0.65;

for i = 1 : size(net.iM,1)
    list = net.neighborsOut(i);
    if length(list)==2 % internal roads
        if isHoriz(i)
            if isHoriz(list(1))
                net.turnings(i,list(1)) = split;
            else
                net.turnings(i,list(1)) = 1 - split;
            end
        else
            if isHoriz(list(1))
                net.turnings(i,list(1)) = 1 - split;
            else
                net.turnings(i,list(1)) = split;
            end
        end
        net.turnings(i,list(2)) = 1-net.turnings(i,list(1));
        
    elseif length(list)==1 % boundary road
        net.turnings(i,list(1)) = 0.3;
    end
        
end
end

