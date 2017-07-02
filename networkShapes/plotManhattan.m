% This file is part of Traffic MacroSimulator.
%
% Traffic MacroSimulator is free software: you can redistribute it and/or
% modify it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% You should have received a copy of the GNU General Public License
% along with Traffic MacroSimulator.  If not, see http://www.gnu.org/licenses/.

% @author: P. Grandinetti

% This scripts create an animation of the Manhattan grid simulation

xLength = 0.1;
yLength = 0.4;
yStep = 0.2;
xStep = 0.2;
shift = 0.01;
xDomain = linspace(0,1,nCols);
yDomain = linspace(-1,0,nRows);

map = colormap(parula);

for t = 1 : time(end)
    disp(t)
    clf, hold on
    for j = 1 : nRows
        
        for i = 1 : nCols-1
            
            road = (j-1)*(nRows+nCols-1) + i;
            
            color = ceil((densities(road,t)*1e3)/2) +1;
            %disp(color)
%             if densities(road,t)>=0.04
%                 color = 'r';
%             else
%                 color = 'g';
%             end
            line([xDomain(i) xDomain(i+1)] + [shift -shift], [yDomain(end+1-j) yDomain(end+1-j)], 'LineWidth',6,'color',map(color,:));
        end
    end
    
    for i = nRows:-1:2
        for j = 1 : nCols
            road = (nRows+nCols-1)*(nRows-i) + nCols-1 + j;
            color = ceil((densities(road,t)*1e3) /2) +1;
%             if densities(road,t)>=0.04
%                 color = 'r';
%             else
%                 color = 'g';
%             end
            line([xDomain(j) xDomain(j)],[yDomain(i) yDomain(i-1)] + [-shift shift], 'LineWidth',6, 'color',map(color,:));
        end
    end
    
    set(gca,'YTickLabel',[]);
    set(gca,'YTick',[]);
    set(gca,'XTickLabel',[]);
    set(gca,'XTick',[]);
    annotation('textbox',[0.485 0.49 0.035 0.02],'String','source')
    title(sprintf('Network densities at t=%0.1f',t))
    colorbar
    drawnow
    
    if t==1
        input('');
    end
    
    pause(0.01)
    
end
