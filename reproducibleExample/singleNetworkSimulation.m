% This file is part of Traffic MacroSimulator.
%
% Traffic MacroSimulator is free software: you can redistribute it and/or
% modify it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% You should have received a copy of the GNU General Public License
% along with Traffic MacroSimulator.  If not, see http://www.gnu.org/licenses/.

% @author: P. Grandinetti

clear;
addpath('./..')
addpath('./../networkShapes')
addpath('./../controllers')

%% FIRST STEP: CREATE THE MATRIX OF THE NETWORK

% It's a matrix with element = 
            %  1 if intersection j is downstream road i,
            %  -1 if intersection j is upstream road i,
            %  0 otherwise
            
% EXAMPLE:
nRows = 4; nCols = 4;
[matrix, turnings] = buildManhattanMultipleInOut(nRows,nCols);
% % OR USE the following for a slightly different shape
% matrix = buildMatrixManhattan(nRows, nCols);
 
% % OR CREATE IT MANUALLY
% % example for highway
% nRoads = 10
% matrix = zeros(nRoads, nRoads+1);
% for i = 1 : nRoads
%     matrix(i,i) = -1;
%     matrix(i,i+1) = 1;
% end


%% SECOND STEP: CREATE THE NETWORK OBJECT

% sampling time
Tc = 15;
% cycle time of traffic lights (must be multiple of Tc)
cycle = 90;

n = Net(Tc,cycle/Tc,(1:size(matrix,1))');
n.iM = matrix;

%% THIRD STEP: DEFINE SPLIT RATIOS

% EXAMPLE
%splitRatiosManhattan(n, nRows, nCols)

% or modify the matrix n.turnings in different ways ...
n.turnings = turnings;

%% FOURTH STEP: DEFINE PARAMETERS OF THE NETWORK AND INITIALIZE IT
% put coherent unit measures
length = 500;
maxSpeed = 50/3.6;
maxDensity = 0.125;
maxFlow = 0.55;
rhoC = maxFlow/maxSpeed;
congSpeed = abs(maxFlow/(rhoC - maxDensity));
type = 'RoadAvgFifoCTM'; % or 'RoadSignFifoCTM'
n.initialize(length, maxSpeed, congSpeed, maxDensity, maxFlow, rhoC, type)

%% FIFTH STEP: DEFINE TRAFFIC LIGHTS IN THE NETWORK
% EXAMPLE: EQUAL TRAFFIC LIGHTS
setTrafficLightsManhattanMultipleIO(n,nCols);

% or define them in different ways...


%% SIXTH STEP: DEFINE SIMULATION PARAMETERS
% time horizon of the simulation
time = [1 n.period*50];
%keyboard
% create demand and supply outside the network
Sout = maxFlow*ones(size(n.iM,1),time(end));
%Din = zeros(size(n.iM,1),time(end));

% CREATE SOURCE OF TRAFFIC FOR THE MANATTHAN GRID
% SEE FILE exampleExternalDemand.m
Din = inputDemandManhattan(n, 0.4, time(end)*0.1, time(end)*0.8, time(end));

% or define them in different ways...


%% NOW RUN SIMULATION AND SAVE RESULTS IN THE WORKSPACE
[densities, flows, semaf, cpuTimes, perf] = n.simul(time,zeros(1,size(n.iM,1)),Din,Sout);

% plot densities time-evolution
imagesc(densities);colorbar;

%% CREATE VIDEO, WORKS ONLY IF IT'S A GRID
%plotManhattan
