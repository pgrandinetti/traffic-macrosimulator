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

% DISCLAIMER: This is a legacy function and might not work together with
% the rest of the package!

function [LightsValues,  densities, fIn, fOut, z, case1, time, objectiveFnc] = MILP3  (net, Din, Sout)

yalmip('clear');
warning('off','YALMIP:strict')

factor = 0.1;

x = sdpvar(length(net.roads),net.period,'full');

demands = sdpvar(length(net.roads), net.period,'full');
supplies = sdpvar(length(net.roads), net.period, 'full');

z = sdpvar(length(net.roads),net.period,'full'); % outflow without lights

fOut = sdpvar(length(net.roads),net.period,'full'); %outflow with lights

fIn = sdpvar(length(net.roads),net.period,'full'); % inflows

case1 = binvar(length(net.roads), net.period,'full');

sigma = sdpvar(length(net.roads),2,'full');
sigmaMin = 2;

F=[];

for i = 1 : size(sigma,1)
    F = F + [sigma(i,1) + sigmaMin <= sigma(i,2)];
    F = F + [1 <= sigma(i,1)<= net.period, 1 <= sigma(i,2) <= net.period];
end

x0 = zeros(length(net.roads),1);
for i = 1 : length(x0)
    x0(i) = net.roads(i).currentDensity;
end

%collison avoidance constraints
for i = 1 : (length(net.roads))
    kin = net.downKin(i);
    for j = 1 : length(kin)
        F = F + [true( sigma(i,1) >= sigma(kin(j),2)+1 | sigma(i,2) <= sigma(kin(j),1)-1 )] ;
    end
end

for k = 1 : size(x,2);
    if k == 1
        xtemp = x0;
    else
        xtemp = x(:,k-1);
    end
    
    for i = 1 : size(demands,1)
        demands(i,k) = min ([ net.roads(i).maxSpeed * xtemp(i), net.roads(i).maxFlow] ); % thanks Yalmip
        supplies(i,k) = min( [ net.roads(i).congSpeed * (net.roads(i).maxDensity - xtemp(i)), net.roads(i).maxFlow ] );
        if k>1
            F = F + [0 <= demands(i,k) <= net.roads(i).maxFlow];
            F = F + [0 <= supplies(i,k)<= net.roads(i).maxFlow];
        end
    end
    
    for i = 1 : size(z,1)
        outRoads = net.neighborsOut(i);
        if ~isempty(outRoads)
            z(i,k) = min ([demands(i,k); supplies(outRoads,k)./net.turnings(:,outRoads)']);
        else
            z(i,k) = min (demands(i,k), Sout(i,k));
        end
        if k>1
            F = F + [0 <= z(i,k)<= net.roads(i).maxFlow];
        end
    end
    
    for i = 1 : size(fOut,1)
        if isempty(net.neighborsOut(i))
            F = F + [case1(i,k)==1];
        else
            F = F + [ iff( k>=sigma(i,1) & k<=sigma(i,2) , case1(i,k)==1 ) ];
            F = F + [ iff( k<sigma(i,1) | k>sigma(i,2) , case1(i,k)==0 ) ];
        end
        
        F = F + [0 <= fOut(i,k)<= z(i,k) , fOut(i,k)<=net.roads(i).maxFlow];
        F = F + [ implies( case1(i,k) , fOut(i,k)==z(i,k)) ];
        F = F + [ implies( ~case1(i,k) , fOut(i,k)==0 ) ];
    end
    
    for i = 1 : size(fIn,1)
        inRoads = net.neighborsIn(i);
        if ~isempty(inRoads)
            fIn(i,k) = net.roads(i).beta * sum(fOut(inRoads,k));
        else
            fIn(i,k) = min (Din(i,k), supplies(i,k));
        end
        if k>1
            F = F + [0 <= fIn(i,k)<= net.roads(i).maxFlow];
        end
    end
    
    for i = 1 : size(x,1)
        F = F + [x(i,k) == xtemp(i) + net.sampleTime * (fIn(i,k) - fOut(i,k))/net.roads(i).L];
        F = F + [0 <= x(i,k) <= [net.roads(i).maxDensity]'];
    end
end

%% Objective functions
innerR = net.enteringRoads();
SoD = min ( net.roads(1).maxFlow*ones(length(innerR),size(x,2)), ...
    net.roads(1).congSpeed * (net.roads(1).maxDensity - x(innerR,:)) );
SoD = min (Din(innerR,1 : size(x,2)), SoD);

TTD = net.roads(1).congSpeed*(net.roads(1).maxDensity - x(setdiff(1:size(x,1),innerR),:));
TTD = min ( TTD, net.roads(1).maxSpeed * x(setdiff(1:size(x,1),innerR),:) );

options = sdpsettings('solver','gurobi','gurobi.MIPGap',0.05,'verbose',0);
sol = optimize(F, -( sum(sum(TTD)) + factor*sum(sum(SoD))) , options);
if sol.problem ~= 0
    error('Numerical error while optimizing')
end
LightsValues = value(sigma);
densities = [x0 value(x)];
fIn = value(fIn);
fOut = value(fOut);
z = value(z);
case1 = value(case1);
time = sol.solvertime;

objectiveFnc = [sum(sum(value(TTD))) sum(sum(value(SoD)))];
end
