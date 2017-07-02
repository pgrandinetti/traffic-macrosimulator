% This file is part of Traffic MacroSimulator.
%
% Traffic MacroSimulator is free software: you can redistribute it and/or
% modify it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% You should have received a copy of the GNU General Public License
% along with Traffic MacroSimulator.  If not, see http://www.gnu.org/licenses/.

% @author: P. Grandinetti

% This class is a modelization of a road network system
% with dynamic model similar to the CTM
% See "Control of large scale traffic networks" by the author, Chapter 3
% Available at https://www.dropbox.com/s/9m2ldonki5x6lfq/ControlLargeScaleTraffic.pdf

classdef Net < handle
    % network with signalizes intersections without storage space
    
    properties (SetAccess = public)
        
        iM; % = dlmread('networkInfo/roads2intersectionMap'); % roads 2 intersection map
            % iM(i,j) = 1 if intersection j is downstream road i,
            %           -1 if intersection j is upstream road i,
            %           0 otherwise
            
        rA; % = dlmread('networkInfo/roadsAdjacencyMatrix'); % roadsAdjacency matrix
            % rA(i,j) = 1 if i->j; -1 if j->i; 0 otherwise
            
        lM; % lights 2 roads maps
            % lM(i) = id of the road associated to traffic light i
            
        turnings; % = dlmread('networkInfo/splitRatios');
                  % turnings(i,j) = percentage of vehicles in i
                  % that want to go in j
        
        sampleTime;
        period;
        roads;
        lights;
        isControlled = 1;
        
        % in future versions intersections may have their dynamic
        % model (possibly with storage space)
        % intersections;
    end
    
    methods 
        
        function N = Net (sampleTime, period, lM)
            N.lM = lM;
            N.sampleTime = sampleTime;
            N.period = period;
        end
        
        function initialize(self, length, maxSpeed, congSpeed, maxDensity, maxFlow, rhoC, type)
            self.roads = [];
            currId = 1;
            dim = max(size(self.rA,1), size(self.iM,1));
            for i = 1:dim
                if strcmp(type,'RoadSignFifoCTM')
                    rTemp = RoadSignFifoCTM(length, maxSpeed, congSpeed, 0, maxDensity, maxFlow, currId);
                elseif strcmp(type, 'RoadAvgFifoCTM')
                    rTemp = RoadAvgFifoCTM(length, maxSpeed, congSpeed, 0, maxDensity, maxFlow, currId);
                end
                rTemp.myNet = self;
                rTemp.criticalDensity = rhoC;
                self.roads = [self.roads rTemp];
                currId = currId + 1;
            end 
            
            currId = 1;
            self.lights = [];
            
            for i=1:max(size(self.lM))
                lTemp = Light(currId, self.period);
                self.lights = [self.lights lTemp];
                currId = currId + 1;
            end
            
            %
            % Here code to initialize intersection if necesary in future
            % release
            %
            % ...
        end
        
        function inRoads = enteringRoads(self)
            inRoads = find(sum(self.iM,2) == 1); 
        end
        
        function outRoads = exitingRoads(self)
            outRoads = find(sum(self.iM,2) == -1);
        end
        
        function neigIn = neighborsIn (self, i)
            % returns roads j such that there exists j-> i
            if ~isempty(self.rA)
                neigIn = find(self.rA(i,:) == -1);
            else
                % use iM
                neigIn = [];
                vec = find (self.iM(i,:) == -1);
                for j = 1 : size(self.iM,1)
                    if j~= i
                        vec2 = find (self.iM(j,:)==1);
                        if size(vec2) == size(vec)
                            if vec2 == vec
                                neigIn = [neigIn j];
                            end
                        end
                    end
                end
            end
        end
        
        function neigOut = neighborsOut (self, i)
            % returns roads j such that there exists i -> j
            if ~isempty(self.rA)
                neigOut = find(self.rA(i,:) == 1);
            else
                % use iM
                neigOut = [];
                vec = find (self.iM(i,:) == 1);
                for j = 1 : size(self.iM,1)
                    if j~= i
                        vec2 = find (self.iM(j,:)==-1);
                        if size(vec2) == size(vec)
                            if vec2 == vec
                                neigOut = [neigOut j];
                            end
                        end
                    end
                end
            end
        end
        
        function k = downKin (self, i)
            % returns roads j connected to the same downstream intersection
            % of i
            k = [];
            if ~isempty(self.iM)
                vec = find (self.iM(i,:) == 1);
                for j = 1 : size(self.iM,1)
                    if j~=i
                        vec2 = find (self.iM(j,:) == 1);
                        if size(vec2) == size(vec)
                            if vec2 == vec
                                k = [k j];
                            end
                        end
                    end
                end
            else
                % use rA
                vec = find (self.rA(i,:) == 1);
                for j = 1:length(self.rA)
                    if j~=i
                        vec2 = find(self.rA(j,:)==1);
                        if size(vec2) == size(vec)
                            if vec2 == vec
                                k = [k j];
                            end
                        end
                    end
                end
            end
        end
        
        function k = upKin (self, i)
            % returns roads j connected to the same upstream intersection
            % of i
            k = [];
            if ~isempty(self.iM)
                vec = find (self.iM(i,:) == -1);
                for j = 1 : size(self.iM,1)
                    if j~=i
                        vec2 = find (self.iM(j,:) == -1);
                        if size(vec2) == size(vec)
                            if vec2 == vec
                                k = [k j];
                            end
                        end
                    end
                end
            else
                % use rA
                vec = find (self.rA(i,:) == -1);
                for j = 1:length(self.rA)
                    if j~=i
                        vec2 = find(self.rA(j,:) == -1);
                        if size(vec2) == size(vec)
                            if vec2 == vec
                                k = [k j];
                            end
                        end
                    end
                end
            end
        end
        
        function k = kinIntersectionDown (self, i)
            % returns all roads connected downstream to intersection i
            k = find(self.iM(:,i) == 1);
        end
        
        function k = kinIntersectionUp (self, i)
            % returns all roads connected upstream to intersection i
            k = find(self.iM(:,i) == -1);
        end
        
        function m = getIncidenceMatrix(self) % nodes by nodes (= intersections)
            m = zeros(size(self.iM,2),size(self.iM,2));
            for i = 1 : size(self.iM,2)
                keyboard
                list = find(self.iM(:,i)== -1); % take roads exiting node i
                for j = 1 : length(list)
                    m(i,find(self.iM(list(j),:)==1))=1;
                end
                list = find(self.iM(:,i)== 1); % take roads entering node i
                for j = 1 : length(list)
                    m(i,find(self.iM(list(j),:)==-1))=-1;
                end
            end
        end
        
        % This is the main function of this object
        % The Net object can "simulate" itself like a dynamical system
        % would do with x(k+1)=f(x(k),u(k))

        function [totDensity, totOutFlows, totLights, computationTimes, performanceIdx] = simul (self, time, x0, Din, Sout)
                      
            %if mod(time(end), net.sampleTime) ~= 0
            %    error('Simulation time must be multiple of sampling time');
            %end
            
            %tf = time(end);
            netSize = max(size(self.rA,1),size(self.iM,1));
            totDensity = [x0' zeros(netSize, time(end))];
            totOutFlows = zeros(netSize,time(end)+1);
            totLights = zeros(size(self.lM,1), time(end));
            computationTimes = zeros(1,ceil(time(end)/self.period));
            performanceIdx = [0 0];
            
            for i = 1 :netSize
                self.roads(i).currentDensity = x0(i);
            end
            
            for k = time(1) : time(end)
                disp(k)

                %% HERE call the controller to compute u[k, k+period-1] 
                 % and set the lights IF THERE IS A TRAFFIC LIGHT SCHEDULER
                if self.isControlled==1 && mod(k,self.period)==1
                    clear('yalmip')
                    % DECOMMENT THE USED CONTROLLER
                    
                    %% Centralized MILP
%                     [sigma, x, fIn, fOut, z, case1, compTime, obj] = MILP3(self, Din(:,k:k+net.period-1),Sout(:,k:k+net.period-1));
%                     computationTimes(ceil(k/net.period)) = compTime;
%                     for i = 1 : size(net.lM, 1)
%                         net.lights(i).values = case1(net.lM(i),:);
%                         totLights(i,k:k+net.period-1) = net.lights(i).values; 
%                     end
%                     performanceIdx = performanceIdx + obj;
                    
                    %% One-step-ahead linear
%                     [LightsValues, xPred, fOut, fIn, duty, obj] = OneStepAheadLP(self, Din(:,k:k+self.period-1), Sout(:,k:k+self.period-1));
%                     for i = 1 : size(self.lM, 1)
%                         self.lights(i).values = LightsValues(i,:); % FOR 1-step-ahead
%                         totLights(i,k:k+self.period-1) = self.lights(i).values; 
%                     end
%                     performanceIdx = performanceIdx + obj;
                    
                    %% Decentralized MILP
%                     [sigma, lightsVal, compTime, obj] = DecMILP2(self, Din(:,k:k+self.period-1), Sout(:,k:k+self.period-1));
%                     for i = 1 : size(net.lM, 1)
%                         net.lights(i).values = lightsVal(i,:);
%                         totLights(i,k:k+net.period-1) = net.lights(i).values;
%                     end
%                     performanceIdx = performanceIdx + obj;
                    
                     %% Centralized Randomized
%                     nSample = 1e3;
%                     [sigma, lightsVal, compTime, obj] =...
%                         Rand2DoF(self, Din(:,k:k+self.period-1), Sout(:,k:k+self.period-1));
%                     for i = 1 : size(net.lM,1)
%                         net.lights(i).values = lightsVal(i,:);
%                         totLights(i,k:k+net.period-1) = net.lights(i).values;
%                     end
%                     performanceIdx = performanceIdx + obj;

                    %% One-step-ahead QP
                   [LightsValues, xPred, fOut, fIn, flows, duty, obj] =...
                       OneStepAheadCentralizedQP(self, Din(:,k:k+self.period-1), Sout(:,k:k+self.period-1) );
                   for i = 1 : size(self.lM, 1)
                       self.lights(i).values = LightsValues(i,:);
                        totLights(i,k:k+self.period-1) = self.lights(i).values; 
                    end
                    performanceIdx = performanceIdx + obj;

                end
                
                fOut = zeros(netSize,1);
                for i = 1 :netSize
                    fOut(i) = self.roads(i).ownOutFlow(Sout, k);
                end
                
                totOutFlows(:,k+2-time(1)) = fOut;
                
                for i = 1 :netSize
                    self.roads(i).updateDensity(fOut, k, Din);
                    totDensity(i,k+2-time(1)) = self.roads(i).currentDensity;
                end
            end
            
        end
        
        function duty = getDutyCycles(self)
            duty = zeros(length(self.lights),1);
            for i = 1 : length(self.lights)  
                duty(i) = self.lights(i).dutyCycle();
            end
        end

    end
end

