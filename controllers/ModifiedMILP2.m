% This file is part of Traffic MacroSimulator.
%
% Traffic MacroSimulator is free software: you can redistribute it and/or
% modify it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% You should have received a copy of the GNU General Public License
% along with Traffic MacroSimulator.  If not, see http://www.gnu.org/licenses/.

% @author: P. Grandinetti

% This function implements a discrete time, distributed, controller for traffic lights
% See "Control of large scale traffic networks" by the author, Chapter 7
% Available at https://www.dropbox.com/s/9m2ldonki5x6lfq/ControlLargeScaleTraffic.pdf

% DISCLAIMER: This is a legacy function and might not work together with
% the rest of the package!

function [sigma, sigmaSugg1, sigmaSugg2, l, t, objective] = ModifiedMILP2( net, i, suggestions, Din, Sout )

% % Modified MILP2 consider suggestions downstream and up(up)stream

yalmip('clear');

roadsIn = net.kinIntersectionDown(i);
roadsOut = net.kinIntersectionUp(i);
roadsInIn = [];
for i = 1 : length(roadsIn)
    roadsInIn = [roadsInIn net.neighborsIn(roadsIn(i))];
end
inner = net.enteringRoads();
sigma = intvar(length(roadsIn),2,'full');
sigmaSugg1 = intvar(length(roadsOut),2,'full');
sigmaSugg2 = intvar(length(roadsInIn),2,'full');

x = sdpvar(length(roadsIn) + length(roadsOut), net.period);

demands = sdpvar(length(roadsIn) + length(roadsOut), net.period);
supplies = sdpvar(length(roadsIn) + length(roadsOut), net.period);
z = sdpvar(length(roadsIn) + length(roadsOut), net.period); % outflow without lights
fOut = sdpvar(length(roadsIn) + length(roadsOut), net.period); % outflow with lights
fIn = sdpvar(length(roadsIn) + length(roadsOut), net.period); % inflows
case1 = binvar(length(roadsIn), net.period);
case2 = binvar(length(roadsOut), net.period);
case3 = binvar(length(roadsInIn), net.period);
sigmaMin = 2;

F=[];
F = F + [sigma(:,1) + sigmaMin <= sigma(:,2)];
F = F + [1 <= sigma(:,1)<= net.period, 1 <= sigma(:,2) <= net.period];
F = F + [sigmaSugg1(:,1) + sigmaMin <= sigmaSugg1(:,2)];
F = F + [1 <= sigmaSugg1(:,1)<= net.period, 1 <= sigmaSugg1(:,2) <= net.period];
F = F + [sigmaSugg2(:,1) + sigmaMin <= sigmaSugg2(:,2)];
F = F + [1 <= sigmaSugg2(:,1)<= net.period, 1 <= sigmaSugg2(:,2) <= net.period];

x0 = zeros(length(roadsIn) + length(roadsOut),1);
for i = 1 : length(roadsIn)
    x0(i) = net.roads(roadsIn(i)).currentDensity;
end
for i = length(roadsIn)+1 : length(roadsIn)+length(roadsOut)
    x0(i) = net.roads(roadsOut(i-length(roadsIn))).currentDensity;
end

%% collison avoidance constraints
ind = 1;
% NB only for roadsIn and roadsInIn (in subgroups)
for i = 1 : length(roadsIn)
    for j = i+1 : length(roadsIn)
        F = F + [true( sigma(i,1) >= sigma(j,2)+1 | sigma(i,2) <= sigma(j,1)-1 )] ;
    end
    len = length(net.neighborsIn(roadsIn(i)));
    for n = ind : ind+len-1
        for m = n+1 : ind+len-1
            F = F + [true( sigmaSugg2(n,1) >= sigmaSugg2(m,2)+1 | sigmaSugg2(n,2) <= sigmaSugg2(m,1)-1 )] ;
        end
    end
    ind = ind + len;
end

%% start predictions
for k = 1 : size(x,2)
    disp(strcat('k= ',num2str(k)))
    if k == 1
        xtemp = x0;
    else
        xtemp = x(:,k-1);
    end
    %% compute D-S for inRoads
    for i = 1 : length(roadsIn)
        r = net.roads(roadsIn(i));
        demands(i,k) = min ([ r.maxSpeed * xtemp(i), r.maxFlow] ); % thanks Yalmip
        supplies(i,k) = min( [ r.congSpeed * (r.maxDensity - xtemp(i)), r.maxFlow ] );
        if k>1
            F = F + [0 <= demands(i,k) <= r.maxFlow];
            F = F + [0 <= supplies(i,k)<= r.maxFlow];
        end
    end
    
    %% compute D-S for outRoads
    for i = length(roadsIn)+1 : length(roadsIn)+length(roadsOut)
        r = net.roads(roadsOut(i-length(roadsIn)));
        demands(i,k) = min ([ r.maxSpeed * xtemp(i), r.maxFlow] ); % thanks Yalmip
        supplies(i,k) = min( [ r.congSpeed * (r.maxDensity - xtemp(i)), r.maxFlow ] );
        if k>1
            F = F + [0 <= demands(i,k) <= r.maxFlow];
            F = F + [0 <= supplies(i,k)<= r.maxFlow];
        end
        %% compute outflows without traff lights
        if ~isempty(roadsOut)
            for i = 1 : length(roadsIn)
                z(i,k) = min ([demands(i,k); supplies(length(roadsIn)+1:end,k)./net.turnings(:,roadsOut)']);
            end
        else
            for i = 1 : length(roadsIn)
                z(i,k) = min(demands(i,k), Sout(roadsIn(i),k));
            end
        end
        for i = 1 : length(roadsOut)
            j = i + length(roadsIn);
            if isempty(net.neighborsOut(roadsOut(i)))
                z(j,k) = min(demands(j,k), Sout(roadsOut(i),k));
            else
                %consider the rest of the network as constant
                minimum = net.roads(roadsOut(i)).maxFlow;
                out = net.neighboursOut(roadsOut(i));
                for n = 1 : length(out)
                    minimum = min ( minimum, net.roads(out(n)).congSpeed * (net.roads(out(n)).maxDensity - net.roads(out(n)).currentDensity) );
                end
                z(j,k) = min ( demands(j,k), minimum );
            end
        end
        
        %% constrain outflows wrt traff lights
        for i = 1 : length(roadsIn)
            F = F + [iff( k>=sigma(i,1) & k<=sigma(i,2), case1(i,k)==1)];
            F = F + [iff( k<=sigma(i,1)-1 | k>=sigma(i,2)+1 , case1(i,k)==0 )];
            F = F + [0 <= fOut(i,k)<= z(i,k) , fOut(i,k)<=net.roads(roadsIn(i)).maxFlow];
            F = F + [implies( case1(i,k), fOut(i,k)==z(i,k))];
            F = F + [implies( ~case1(i,k), fOut(i,k)==0 )];
        end
        
        for i = 1 : length(roadsOut)
            if isempty(net.neighborsOut(roadsOut(i)))
                F = F + [ case2(i,k)==1 ];
            else
                F = F + [ iff( k>=sigmaSugg1(i,1) & k<=sigmaSugg1(i,2) , case2(i,k)==1 ) ];
                F = F + [ iff( k<=sigmaSugg1(i,1)-1 | k>=sigmaSugg1(i,2)+1 , case2(i,k)==0 ) ];
            end
            j = i + length(roadsIn);
            F = F + [0 <= fOut(j,k)<= z(j,k) , fOut(j,k)<=net.roads(roadsOut(i)).maxFlow];
            F = F + [ implies( case2(i,k) , fOut(j,k)==z(j,k)) ];
            F = F + [ implies( ~case2(i,k) , fOut(j,k)==0 ) ];
        end
        
        %% constrain case3 wrt sigmaSugg2
        for i = 1 : length(roadsInIn)
            F = F + [ iff( k>=sigmaSugg2(i,1) & k<=sigmaSugg2(i,2) , case3(i,k)==1 ) ];
            F = F + [ iff( k<=sigmaSugg2(i,1)-1 | k>=sigmaSugg2(i,2)+1 , case3(i,k)==0 ) ];
        end
        
        
        %% compute fIn
        for i = 1 : length(roadsOut)
            beta = sum(net.turnings(:,roadsOut(i)))/length(net.neighborsIn(roadsIn(i)));
            fIn(i+length(roadsIn),k) = beta * sum(fOut(1:length(roadsIn),k));
        end
        ind = 0;
        for i = 1 : length(roadsIn)
            % use case3
            beta = sum(net.turnings(:,roadsIn(i)))/length(net.neighborsIn(roadsIn(i)));
            in = net.neighboursIn(roadsIn(i));
            if isempty(in)
                fIn(i,k) = min(Din(roadsIn(i),k), supplies(i,k));
            else
                fTemp = sdpvar(length(in),1);
                F = F + [0 <= fTemp <= net.roads(i).maxFlow];
                for n = 1 : length(in)
                    F = F + [ 0 <= fTemp(n) <= min([supplies(i,k)/beta, net.roads(in(n)).maxSpeed * net.roads(in(n)).currentDensity, net.roads(i).maxFlow]) ];
                    F = F + [ implies( case3(ind+n,k) , fTemp(n) >= min([supplies(i,k)/beta, net.roads(in(n)).maxSpeed * net.roads(in(n)).currentDensity, net.roads(i).maxFlow]) )];
                    F = F + [ implies( ~case3(ind+n,k) , fTemp(n) <= 0 ) ];
                end
                ind = ind + length(in);
                fIn(i,k) = beta * sum(fTemp);
            end
        end
        
        %% update density
        %F = F + [0 <= x(:,k) <= [net.roads([roadsIn; roadsOut]).maxDensity]' ];
        for i = 1 : size(x,1)
            F = F + [x(i,k) == xtemp(i) + net.sampleTime * (fIn(i,k) - fOut(i,k))/net.roads(i).L];
            F = F + [0 <= x(i,k) <= net.roads(i).maxDensity];
        end
    end
    
    %% optimization
    [indexSoD, ia, ib] = intersect(roadsIn,inner);
    disp(strcat('indexSoD ', num2str(indexSoD)))
    
    SoD = min ( net.roads(1).maxFlow*ones(length(indexSoD),size(x,2)), ...
        net.roads(1).congSpeed * (net.roads(1).maxDensity - x(ia,:)) );
    SoD = min (Din(indexSoD,:), SoD);
    [remain, ib] = setdiff([roadsIn;roadsOut],indexSoD);
    TTD =  min ( net.roads(1).maxSpeed * x(ib,:), ...
        net.roads(1).congSpeed*(net.roads(1).maxDensity - x(ib,:)) );
    
    factor = 0.1;
    factor2 = 200;
    
    obj = -(sum(sum(TTD)) + factor*sum(sum(SoD)));
    
    if ~isempty(suggestions)
        a = zeros(length(roadsIn),2);
        for i = 1 : length(roadsIn)
            count = 0;
            for n = 1 : 2 : size(suggestions,2)
                if ~isequal(suggestions(roadsIn(i),n:n+1),[-1 -1])
                    a(i,:) = a(i,:) + suggestions(roadsIn(i),n:n+1);
                    count = count + +1;
                end
            end
            a(i,:) = a(i,:)/count;
        end
        b = zeros(length(roadsOut),2);
        for i = 1 : length(roadsOut)
            count = 0;
            for n = 1 : 2 : size(suggestions,2)
                if ~isequal(suggestions(roadsOut(i),n:n+1),[-1 -1])
                    b(i,:) = b(i,:) + suggestions(roadsOut(i),n:n+1);
                    count = count + +1;
                end
            end
            b(i,:) = b(i,:)/count;
        end
        c = zeros(length(roadsInIn),2);
        for i = 1 : length(roadsInIn)
            count = 0;
            for n = 1 : 2 : size(suggestions,2)
                if ~isequal(suggestions(roadsInIn(i),n:n+1),[-1 -1])
                    c(i,:) = c(i,:) + suggestions(roadsInIn(i),n:n+1);
                    count = count + +1;
                end
            end
            c(i,:) = c(i,:)/count;
        end
        obj = obj + factor2*(norm([sigma; sigmaSugg1; sigmaSugg2]-[a;b;c],1));
        %obj = (norm([sigma; sigmaSugg1; sigmaSugg2]-[a;b;c],1));
    end
    
    options = sdpsettings('solver','gurobi','verbose',0);
    sol = optimize(F, obj , options);
    if sol.problem ~= 0
        error('Numerical error while optimizing')
    end
    
    sigma = value(sigma);
    sigmaSugg1 = value(sigmaSugg1);
    sigmaSugg2 = value(sigmaSugg2);
    l = value(case1);
    t = sol.solvertime;
    
    objective = [value(sum(sum(TTD))) value(sum(sum(SoD)))];
end


