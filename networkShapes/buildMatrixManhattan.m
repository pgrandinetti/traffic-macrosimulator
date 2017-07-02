% This file is part of Traffic MacroSimulator.
%
% Traffic MacroSimulator is free software: you can redistribute it and/or
% modify it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% You should have received a copy of the GNU General Public License
% along with Traffic MacroSimulator.  If not, see http://www.gnu.org/licenses/.

% @author: P. Grandinetti

function A = buildMatrixManhattan(nRows, nCols)

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

end

