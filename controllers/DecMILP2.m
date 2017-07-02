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

function [sigma, lights, compTime, objective] = DecMILP2  (net, Din, Sout)
compTime = 0;

%DEC-MILP consider suggestions downstream and up(up)stream

Nstep = size(net.iM,2); % num of intersections
%Nstep = size(net.iM,2)/2;
%Nstep = 1;

lights = zeros(size(net.iM,1),net.period);

sigma = -ones(size(net.iM,1),2); % lights for exiting roads will be all green
%sigmaSugg1 = -ones(size(net.iM,1),2); % suggested downstream values for agreement
sigmaSugg = -ones(size(net.iM,1),2*size(net.iM,2)); % suggested upstream values for agreement
% element(i,j) is the
% sigma suggested by
% intersection j for
% traff light i (i.e.
% is a 2 value
% vector)

%% set values for exiting roads
for i = 1 : size(net.iM,1)
    if isempty(net.neighboursOut(i))
        sigma(i,:) = [1 net.period];
        lights(i,:) = ones(1,net.period);
    end
end

%% solve first iteration
for i = 1 : size(net.iM,2)
    [sigmaRes, sigmaSuggRes1, sigmaSuggRes2, l, objective] = ModifiedMILP2(net,i,[], Din, Sout);
    sigma(net.kinIntersectionDown(i),:) = sigmaRes;
    sigmaSugg(net.kinIntersectionUp(i),2*i-1:2*i) = sigmaSuggRes1;
    inRoads = net.kinIntersectionDown(i);
    index = 1;
    for j = 1 : length(inRoads)
        inInRoads = net.neighborsIn(inRoads(j));
        for k = 1 : length(inInRoads)
            sigmaSugg(inInRoads(k),2*i-1:2*i) = sigmaSuggRes2(index,:);
            index = index+1;
        end
    end
    lights(inRoads,:) = l;
end

disp(sigma)
disp(sigmaSugg)


%     %% set values for traff lights without downstream intersection
%      for i = 1 : size(net.iM,1)
%          if isempty(net.neighboursOut(i))
%              lights(i,:) = ones(1,net.period);
%          end
%      end

%% iterate
for k = 1 : Nstep
    disp(strcat('Num iter = ',num2str(k)))
    %stop-criteria
    stop = 1;
    for i = 1 : size(net.iM,1)
        disp(i)
        for n = 3 : 2 : size(net.iM,2)
            if ~isequal(sigmaSugg(i,1:2), sigmaSugg(i,n:n+1))
                stop = 0;
                break
            end
        end
    end
    if stop == 1
        return
    end
    
    for i = 1 : size(net.iM,2)
        %suggestions2 = sigmaSugg2(all(sigmaSugg2(:,2*i-1:2*i)~=-1,2),2*i-1,2*i); %magic :-)
        [sigmaRes, sigmaSuggRes1, sigmaSuggRes2, l, t, objective] = ModifiedMILP2(net, i, sigmaSugg, Din, Sout);
        compTime = compTime + t;
        sigma(net.kinIntersectionDown(i),:) = sigmaRes;
        sigmaSugg(net.kinIntersectionUp(i),2*i-1:2*i) = sigmaSuggRes1;
        inRoads = net.kinIntersectionDown(i);
        index = 1;
        for j = 1 : length(inRoads)
            inInRoads = net.neighborsIn(inRoads(j));
            for n = 1 : length(inInRoads)
                sigmaSugg(inInRoads(n),2*i-1:2*i) = sigmaSuggRes2(index,:);
                index = index+1;
            end
        end
        lights(inRoads,:) = l;
    end
    
    %% set values for traff lights without suggestions
    %         for i = 1 : size(net.iM,1)
    %             if isempty(net.neighboursIn(i))
    %                 sigmaSugg(i,:) = repmat(sigma(i,:),1,size(sigmaSugg,2)/2);
    %             end
    %         end
    
    disp(sigma)
    disp(sigmaSugg)
    
end
end
