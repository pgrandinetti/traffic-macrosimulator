% This file is part of Traffic MacroSimulator.
%
% Traffic MacroSimulator is free software: you can redistribute it and/or
% modify it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% You should have received a copy of the GNU General Public License
% along with Traffic MacroSimulator.  If not, see http://www.gnu.org/licenses/.

% @author: P. Grandinetti

% This function implements a discrete time controller for traffic lights
% See "Control of large scale traffic networks" by the author, Chapter 7
% Available at https://www.dropbox.com/s/9m2ldonki5x6lfq/ControlLargeScaleTraffic.pdf

function [n, duty, LightValues] = distributedQP(net,Din,Sout)

nIterMax = 1e3;
globalTime = 1;
alpha = 0.1;

% create list of agent
agentsList = [];
for k = 1 : length(net.roads)
    agentsList= [agentsList LocalAgent6(k, net)];
end

for n = 1 : nIterMax
    disp(n)
    % primal update;
    parfor k = 1 : length(net.roads)
        asd = agentsList(k);
        asd.primalUpdate(net, mean(Din,2), Sout, globalTime);
        agentsList(k).primalVarU = asd.primalVarU;
        agentsList(k).primalVarY = asd.primalVarY;
    end
    % share and collect variables
    for k = 1 : length(net.roads)
        agentsList(k).collectNeigVar(agentsList);
    end
    % dual update
    for k = 1 : length(net.roads)
        agentsList(k).dualUpdate(alpha);
    end
    %check convergence
    stop = 1;
    for j = 1 : length(net.roads)
        if stop == 1 && agentsList(j).isConverged == 0
            stop = 0;
        end
    end
    if stop == 1
        break;
    end
    
end

%agentsList(:).primalVarU
duty = arrayfun(@(x) agentsList(x).primalVarU(1), 1:length(agentsList));
LightValues = convertDutyCycles(net,duty);
end
