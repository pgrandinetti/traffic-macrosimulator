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
% See "Control of large scale traffic networks" by the author, Chapter 5
% Available at https://www.dropbox.com/s/9m2ldonki5x6lfq/ControlLargeScaleTraffic.pdf

function [LightsValues, xPred, fOut, fIn, duty, objective] = OneStepAheadLP(net, Din, Sout)
    yalmip('clear')
    duty = sdpvar(size(net.lM,1),1);
    F = [duty(net.exitingRoads)==1];
    F= F + [duty>=0.1, duty <=1];   
    
    sdp_fOut = sdpvar(length(net.roads),1);
    sdp_fIn = sdpvar(length(net.roads),1);
    
    demands = zeros(length(net.roads),1);
    supplies = zeros(length(net.roads),1);

    xCurr = [net.roads(:).currentDensity];
    
    for i = 1 : size(net.lM,1)
        demands(i) = min(net.roads(i).maxSpeed * xCurr(i), net.roads(i).maxFlow);
        supplies(i) = min(net.roads(i).maxFlow, net.roads(i).congSpeed * (net.roads(i).maxDensity - xCurr(i))); 
    end
    
    for i = 1 : length(net.roads)
        outRoads = net.neighborsOut(i);
        if ~isempty(outRoads)
            sdp_fOut(i) = duty(net.lM==i) * min([demands(i); (supplies(outRoads)./net.turnings(i,outRoads)')]);
        else
            sdp_fOut(i) = min(demands(i), Sout(i,1)); 
        end
    end
    
    for i = 1 : length(net.roads)
        inRoads = net.neighborsIn(i);
        if ~isempty(inRoads)
            sdp_fIn(i) = dot(net.turnings(inRoads,i), sdp_fOut(inRoads));
        else
            sdp_fIn(i) = min(Din(i), supplies(i));
        end
    end
    
    for i = 1 : size(net.iM , 2)
        kin = net.kinIntersectionDown(i);
        if ~isempty(kin)
            F = F + [sum(duty(kin))<=1];
        end
    end
    
    xPred = xCurr' + (net.sampleTime./[net.roads(:).L]').*(sdp_fIn - sdp_fOut);
    innerR = net.enteringRoads();
    factor = 0.1;
    
    SoD = min ( net.roads(1).maxFlow*ones(length(innerR),size(xPred,2)), ...
        net.roads(1).congSpeed * (net.roads(1).maxDensity - xPred(innerR,:)) );
    SoD = min ( Din(innerR,1 : size(xPred,2)), SoD );

    TTD = min ( net.roads(1).maxSpeed * xPred(setdiff(1:size(xPred,1),innerR),:), ...
        net.roads(1).congSpeed*(net.roads(1).maxDensity - xPred(setdiff(1:size(xPred,1),innerR),:)) ); % TTD 
    
    obj = sum(sum(TTD)) + factor * sum(sum(SoD));
    options = sdpsettings('verbose',0); % 'solver', 'mosek', 
    sol = optimize(F, -obj , options);
    if sol.problem ~= 0
        disp(sol)
        error('Numerical error while optimizing')
    end
    duty = value(duty);
    LightsValues = convertDutyCycles(net, duty);   
    xPred = value(xPred);
    fIn = value(sdp_fIn);
    fOut = value(sdp_fOut);
    objective = [value(sum(sum(TTD))) value(sum(sum(SoD)))];
end
