% This file is part of Traffic MacroSimulator.
%
% Traffic MacroSimulator is free software: you can redistribute it and/or
% modify it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% You should have received a copy of the GNU General Public License
% along with Traffic MacroSimulator.  If not, see http://www.gnu.org/licenses/.

% @author: P. Grandinetti

function [A,turnings] = buildManhattanMultipleInOut(nRows, nCols)
% Builds a manhattan-like network but with multiple I/O

nRoads = nRows*(nCols+1)+nCols*(nRows+1);
nInters = nRows*nCols;
A = zeros(nRoads,nInters);
turnings = zeros(nRoads,nRoads);

split = 0.65;

used = @(r) r*nCols+(r-1)*(nCols+1);
%areConnected = @(n1,n2,A) find(A(n1,:)==1)==find(A(n2,:)==-1) || find(A(n1,:)==-1)==find(A(n2,:)==1);

for r = 1:nRows
    for c = 1:nCols
        j = (r-1)*nCols+c;
        h1 = used(r)+c;
        h2 = used(r)+c+1;
        v1 = used(r)-(nCols-c);
        v2 = used(r+1)-(nCols-c);
        if mod(r,2)==0
            markH = h2;
            A(h1,j) = -1;
            A(h2,j) = 1;
            turnings(h2,h1) = split;
        else
            markH = h1;
            A(h1,j) = 1;
            A(h2,j) = -1;
            turnings(h1,h2) = split;
        end
        if mod(c,2)==0
            markV = v2;
            A(v1,j) = -1;
            A(v2,j) = 1;
            turnings(v2,v1) = split;
        else
            markV = v1;
            A(v1,j) = 1;
            A(v2,j) = -1;
            turnings(v1,v2) = split;
        end
        turnings(markH, setdiff([v1 v2],markV)) = 1-split;
        turnings(markV, setdiff([h1 h2],markH)) = 1-split;
    end
end

end

