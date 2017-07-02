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

function [LightsValues, xPred, fOut, fIn, flows, duty, objective] = OneStepAheadCentralizedQP(net, Din, Sout)
    yalmip('clear')

    duty = sdpvar(length(net.lM),1);
    duty0 = net.getDutyCycles();
    F = [duty(net.exitingRoads)==1];
    F= F + [duty>=0.1, duty <=1];
    
    DinVar = mean(Din,2);
    SoutVar = mean(Sout,2);

    fOut = sdpvar(length(net.roads),1);
    fIn = sdpvar(length(net.roads),1);

    demands = zeros(length(net.roads),1);
    supplies = zeros(length(net.roads),1);

    xCurr = [net.roads(:).currentDensity];

    for i = 1 : length(net.roads)
        demands(i) = min(net.roads(i).maxSpeed * net.roads(i).currentDensity, net.roads(i).maxFlow);
        supplies(i) = min (net.roads(i).maxFlow, net.roads(i).congSpeed * (net.roads(i).maxDensity - net.roads(i).currentDensity ) );
    end

    for i = 1 : length(net.roads)
        outRoads = net.neighborsOut(i);
        if ~isempty(outRoads)
            fOut(i) = duty(net.lM==i) * min([demands(i); (supplies(outRoads)./net.turnings(i,outRoads)')]);
        else
            fOut(i) = duty(net.lM==i) * min(demands(i), SoutVar(i));
        end
    end

    for i = 1 : length(net.roads)
        inRoads = net.neighborsIn(i);
        if ~isempty(inRoads)
            fIn(i) = dot( net.turnings(inRoads,i), fOut(inRoads));
        else
            fIn(i) = min(DinVar(i), supplies(i));
        end
    end

    for i = 1 : size(net.iM , 2)
        kin = net.kinIntersectionDown(i);
        if ~isempty(kin)
            F = F + [sum(duty(kin))==1];
        end
    end

    xPred = xCurr' + net.sampleTime./[net.roads(:).L]' .* (fIn - fOut);
    bal = 0;
    for i = 1 : length(net.roads)
        iNeighDown = net.neighborsOut(i);
        for j = 1 : length(iNeighDown)
            tmp = sdpvar(1);
            F = F + [tmp == xPred(i) - xPred(iNeighDown(j))];
            bal = bal + tmp^2;
            %bal = bal + tmp^2/(net.roads(i).maxDensity*0.5);
            %bal = bal + (xPred(i) - xPred(iNeighDown(j)))^2;
        end
    end

    %innerR = net.innerRoads();
    %factor = sum([net.roads(:).maxFlow]);
    TTD = min ( net.roads(1).maxSpeed * xPred, ...
        net.roads(1).congSpeed*(net.roads(1).maxDensity - xPred) ); % TTD
    %obj = bal- sum(TTD./([net.roads(:).maxFlow]')) + (duty-duty0)'*(duty-duty0);
    %obj = bal- sum(TTD./[net.roads(:).maxFlow]');
    %obj = - sum(TTD) + (duty-duty0)'*(duty-duty0);
    
    obj = bal/net.roads(1).maxDensity- sum(TTD)/net.roads(1).maxFlow;% + (duty-duty0)'*(duty-duty0);
    sol = optimize(F, obj, sdpsettings('verbose', 0)); % 'solver', 'mosek', 
    if sol.problem ~= 0
        disp(sol)
        error('Numerical error while optimizing')
    end
    duty = value(duty);
    LightsValues = convertDutyCycles(net, duty);
    xPred = value(xPred);
    fIn = value(fIn);
    fOut = value(fOut);
    flows = value(TTD);
    objective = [value(sum(sum(bal))) value(sum(sum(TTD)))];

end