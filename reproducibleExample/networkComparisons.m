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

nRows = 4; nCols = 4;
[matrix, turnings] = buildManhattanMultipleInOut(nRows,nCols);
Tc = 15;
cycle = 90;
lengthRoad = 500;
maxSpeed = 50/3.6;
maxDensity = 0.125;
maxFlow = 0.55;
rhoC = maxFlow/maxSpeed;
congSpeed = abs(maxFlow/(rhoC - maxDensity));
type = 'RoadSignFifoCTM';
time = [1 (cycle/Tc)*50];
Sout = maxFlow*ones(size(matrix,1),time(end));

% network with controlled traffic lights
netControl = Net(Tc,cycle/Tc,(1:size(matrix,1))');
netControl.isControlled = 1;
netControl.iM = matrix;
netControl.turnings = turnings;
netControl.initialize(lengthRoad, maxSpeed, congSpeed, maxDensity, maxFlow, rhoC, type)
setTrafficLightsManhattanMultipleIO(netControl,nCols);

burstValue = 0.4; t0 = time(end)*0.1; tf = time(end)*0.8;
Din = inputDemandManhattan(netControl, burstValue, t0, tf, time(end));

[totDensityC, ~,totLightsC,~,~] = netControl.simul(time,zeros(1,size(netControl.iM,1)),Din,Sout);

% network without control
netNoControl = Net(Tc,cycle/Tc,(1:size(matrix,1))');
netNoControl.isControlled = 0;
netNoControl.iM = matrix;
netNoControl.turnings = turnings;
netNoControl.initialize(lengthRoad, maxSpeed, congSpeed, maxDensity, maxFlow, rhoC, type)
setTrafficLightsManhattanMultipleIO(netNoControl,nCols);

[totDensityNC, ~,totLightsNC,~,~]=netNoControl.simul(time,zeros(1,size(netControl.iM,1)),Din,Sout);

TTD = zeros(1,size(totDensityC,2));
TTDNC = zeros(1,size(totDensityC,2));

for i = 1 : size(totDensityC,2)
    
    for j = 1 : size(totDensityC,1)
        TTD(i) = TTD(i) + min(totDensityC(j,i)*netControl.roads(j).maxSpeed,...
            netControl.roads(j).congSpeed*(netControl.roads(j).maxDensity - totDensityC(j,i)));
        
        TTDNC(i) = TTDNC(i) + min(totDensityNC(j,i)*netNoControl.roads(j).maxSpeed,...
            netNoControl.roads(j).congSpeed*(netNoControl.roads(j).maxDensity - totDensityNC(j,i)));
    end
    
    if i>1
        TTD(i) = TTD(i) + TTD(i-1);
        TTDNC(i) = TTDNC(i) + TTDNC(i-1);
    end
end

figure; hold on;
plot(1:size(totDensityC,2), TTD, 'b');
plot(1:size(totDensityC,2), TTDNC, 'r');
legend('TTD with feedback', 'TTD');

BAL = zeros(1,size(totDensityC,2));
BALNC = zeros(1,size(totDensityC,2));
Lap = zeros(size(totDensityC,1));
for i = 1 : size(Lap,1)
    for j = 1 : size(Lap,1)
        disp(i)
        disp(j)
        if i==j
            Lap(i,j) = length(netControl.neighborsIn(i))+length(netControl.neighborsOut(i));
        elseif any([netControl.neighborsIn(i) netControl.neighborsOut(i)]==j)
            Lap(i,j) = -1;
        end
    end
end

for i = 1 : size(totDensityC,2)
    BAL(i) = totDensityC(:,i)'*Lap*totDensityC(:,i);
    BALNC(i) = totDensityNC(:,i)'*Lap*totDensityNC(:,i);
end

figure; hold on;
plot(1:size(totDensityC,2), BAL, 'b');
plot(1:size(totDensityC,2), BALNC, 'r');
legend('Balancing with feedback', 'Balancing');

figure;
imagesc(abs(totDensityC));h=colorbar; %set(h, 'ylim', [0,maxDensity])
title('Controlled network'); xlabel('time step');
figure;
imagesc(abs(totDensityNC)); h=colorbar; %set(h, 'ylim', [0,maxDensity])
title('Uncontrolled network'); xlabel('time step');

figure; hold on; grid;
SoD = zeros(length(netControl.enteringRoads),tf-t0);
SoDNC = zeros(size(SoD));
for i = 1 : size(SoD,2)
    for j = 1 : size(SoD,1)
        SoD(j,i) = min([Din(j,i), netControl.roads(j).maxFlow,...
            netControl.roads(j).congSpeed*(netControl.roads(j).maxDensity - totDensityC(j,i))]);
        SoDNC(j,i) = min([Din(j,i), netNoControl.roads(j).maxFlow,...
            netNoControl.roads(j).congSpeed*(netNoControl.roads(j).maxDensity - totDensityNC(j,i))]);
    end
end
plot(1:(tf-t0), sum(SoD), 'b')
plot(1:(tf-t0), sum(SoDNC), 'r')
xlabel('time step')
legend('SoD with feedback', 'SoD');
