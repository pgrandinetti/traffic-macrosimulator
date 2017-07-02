% This file is part of Traffic MacroSimulator.
%
% Traffic MacroSimulator is free software: you can redistribute it and/or
% modify it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% You should have received a copy of the GNU General Public License
% along with Traffic MacroSimulator.  If not, see http://www.gnu.org/licenses/.

% @author: P. Grandinetti

function net = buildManhattanGrid(nRows, nCols)

isHoriz = @(i) mod(i, nRows+nCols-1)<=nCols-1 & mod(i, nRows+nCols-1)>=1;
A = zeros(nRows*(nCols-1)+nCols*(nRows-1),nRows*nCols); 
% build the road2node matrix for the manhattan grid
for i = 1 : nRows*nCols
    j = ceil(i/nCols);
    k = i-(j-1)*nCols;
    if j > 1
        if mod(k,2)==0
            A( (nRows+nCols-1)*(j-2)+nCols-1+k, i ) = 1;
        else
            A( (nRows+nCols-1)*(j-2)+nCols-1+k, i ) = -1;
        end
    end
    if j<nRows
        if mod(k,2)==0
            A( (nRows+nCols-1)*(j-1)+nCols-1+k, i ) = -1;
        else
            A( (nRows+nCols-1)*(j-1)+nCols-1+k, i ) = 1;
        end
    end
    if k>1
        if mod(j,2)==0
            A ( (nRows+nCols-1)*(j-1)+k-1, i ) = -1;
        else
            A ( (nRows+nCols-1)*(j-1)+k-1, i ) = 1;
        end
    end
    if k<nCols
        if mod(j,2)==0
            A ( (nRows+nCols-1)*(j-1)+k, i ) = 1;
        else
            A ( (nRows+nCols-1)*(j-1)+k, i ) = -1;
        end
    end
end
% in hours
Tc = 1/3600;
net = Net(Tc,90,(1:size(A,1))');
net.iM = A;
% in km and hours
net.initialize(0.5,50,33.3,125,2500,50, 'RoadSignFifoCTM')
net.turnings = zeros(size(A,1),size(A,1));
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
%time = [1 net.period*50];
% Sout = 0.55*ones(length(net.iM),time(end));
% Din = zeros(length(net.iM),time(end));

% set equal traffic lights
for i = 1 : size(net.iM,1)
    list = net.downKin(i);
    if ~isempty(list)
        if isHoriz(i)
            net.lights(i).values = [ones(1,net.period/2) zeros(1,net.period/2)];
        else
            net.lights(i).values = [zeros(1,net.period/2) ones(1,net.period/2)];
        end
    else
        net.lights(i).values = ones(1,net.period);
    end
    
end
end
