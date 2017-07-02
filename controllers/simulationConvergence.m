% This file is part of Traffic MacroSimulator.
%
% Traffic MacroSimulator is free software: you can redistribute it and/or
% modify it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% You should have received a copy of the GNU General Public License
% along with Traffic MacroSimulator.  If not, see http://www.gnu.org/licenses/.

% @author: P. Grandinetti

% This script run a test-bench of simulations to analyze the convergence
% speed of the distributed algorithm described in
% "Control of large scale traffic networks" by the author, Chapter 7
% Available at https://www.dropbox.com/s/9m2ldonki5x6lfq/ControlLargeScaleTraffic.pdf

clear;
addpath('./networkShapes')

rng(1,'twister') % to reproduce the simulation

%nInters = [1 4 9 16 25 36 49];
%nRoads = [4 12 24 40 60 84 112];% 2*(sqrt(nInters)+1)*sqrt(nInters)

nRows = 3;
nCols = 3;

nSimul = 1; % num of simulation for every network's dimension

% save: centralized solution, distributed solution, num of iteration to
% converge
statistics = cell(length(nRows), nSimul, 3);

% IMPORTANT:
% eveything is in hours or kilometers

% lgo info
%fId = fopen('logInfo', 'wt');

for i = 1 : length(nRows)
    for n = 1 : nSimul
        %clear java
        clearvars -except i n nRows nCols nSimul statistics fId
        clear('yalmip')
        disp(strcat(['nRows = ', int2str(nRows(i)),' Simulation number ',int2str(n)]))
        %fprintf(fId,strcat(['nRows = ', int2str(nRows(i)),' Simulation number ',int2str(n)], '\n'));
        
        net = buildManhattanGrid(nRows(i),nCols(i));
       
        % light traffic conditions
        Din = (rand(length(net.iM),1)*0.5+0.5)*1000 .* ones(length(net.iM),1);
        Sout = 2000*ones(length(net.iM),1);
        
        % free flow condition
        %for j = 1 :length(net.roads)
        %    net.roads(j).currentDensity = rand*net.roads(j).criticalDensity;
        %end
        
        % congested condition
        %for j = 1 :length(net.roads)
        %    net.roads(j).currentDensity = rand*(net.roads(j).maxDensity-net.roads(j).criticalDensity)...
        %        +net.roads(j).criticalDensity;
        %end
        
        % mixed condition
        % density between 20% and 80% of maximum density
        for j = 1 : length(net.roads)
            low = 0.2*net.roads(j).maxDensity;
            up = 0.8*net.roads(j).maxDensity;
            net.roads(j).currentDensity = rand*(up-low) + low;
        end
        
        [LightsValues, xPred, fOut, fIn, flows, duty, objective] = OneStepAheadCentralizedQP(net, Din, Sout);
        
        statistics{i,n,1} = duty;

        [iterDistr, dutyDistr, valuesDistr] = distributedQP(net, Din, Sout);
        
        disp(strcat(['Num of iterations ',int2str(iterDistr)]))
        statistics{i,n,2} = dutyDistr;
        statistics{i,n,3} = iterDistr;
        
        save savingsCongested.mat i n statistics 
    end
end