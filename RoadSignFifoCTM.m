% This file is part of Traffic MacroSimulator.
%
% Traffic MacroSimulator is free software: you can redistribute it and/or
% modify it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% You should have received a copy of the GNU General Public License
% along with Traffic MacroSimulator.  If not, see http://www.gnu.org/licenses/.

% @author: P. Grandinetti

% This class is a modelization of a traffic road system with binary valued
% traffic lights and dynamic model similar to the CTM
% See "Control of large scale traffic networks" by the author, Chapter 3
% Available at https://www.dropbox.com/s/9m2ldonki5x6lfq/ControlLargeScaleTraffic.pdf


classdef RoadSignFifoCTM < handle
    properties (SetAccess = public)
        myNet; % a pointer to the network object this road belongs to
        L;
        maxSpeed; %v
        congSpeed; %w
        currentDensity;
        id;
        maxDensity;
        maxFlow;
        criticalDensity;
        computedOutFlow; %-1 when the outflow has not been computed yes by this road
    end
    
    methods 
        
        function R = RoadSignFifoCTM (length, maxSpeed, congSpeed, currentDensity, maxDensity, maxFlow, id)
            R.L = length;
            R.maxSpeed = maxSpeed;
            R.congSpeed = congSpeed;
            R.currentDensity = currentDensity;
            R.id = id;
            R.maxDensity = maxDensity;
            R.maxFlow = maxFlow;    
            R.myNet = [];
            R.criticalDensity = [];
            R.computedOutFlow = -1;
        end
        
        function supply = getSupply(self)
            supply = min( self.maxFlow,...
                self.congSpeed * (self.maxDensity - self.currentDensity) );
        end
        
        
        function demand = getDemand(self)
            demand = min (self.maxSpeed * self.currentDensity , self.maxFlow);
        end
        
        % Given the outflows computed by each road and the external demand,
        % the road can update its density (i.e., computing its state at the
        % next time instant)
        function updateDensity (self, fOut, time, Din)
            inRoads = self.myNet.neighborsIn(self.id);
            myLight = find (self.myNet.lM == self.id);
            if isempty(myLight)
                myColor = 1;
            else
                myColor = self.myNet.lights(myLight).valueAt(time);
            end
            if isempty(inRoads)
                inF = min (self.getSupply, Din(self.id, time));
            else
                sem = self.myNet.lights(inRoads);
                color = zeros(1,length(sem));
                for k = 1 : length(color)
                    color(k) = sem(k).valueAt(time);
                end
                inF = color*(fOut(inRoads).*self.myNet.turnings(inRoads,self.id))...
                    + min (self.getSupply, Din(self.id, time));
            end
            
            self.currentDensity = self.currentDensity + ...
                self.myNet.sampleTime * ...
                (inF - fOut(self.id)*myColor)/ self.L;
            
            if self.currentDensity < 0
                error('Something went wrong in the simulation: you have a road with negative density. Remember that in the discrete CTM you need to have Ts*v/L < 1');
            end
            if self.currentDensity > self.maxDensity
                error('Something went wrong in the simulation: you have a road with density above threshold');
            end
            self.computedOutFlow = -1;
        end
        
        % Given the current network's state, each road is able to
        % compute its own outflow of vehicles.
        function f = ownOutFlow (self, Sout, time)
            if self.computedOutFlow ~= -1
                f = self.computedOutFlow;
            else
                outRoads = self.myNet.neighborsOut(self.id);
                minimum = 1e10;
                if length(outRoads)>=1
                    minimum = min( minimum,...
                        self.myNet.roads(outRoads(1)).getSupply/self.myNet.turnings(self.id, outRoads(1)));
                    
                    for k = 2 : length(outRoads)
                        minimum = min (minimum,...
                            self.myNet.roads(outRoads(k)).getSupply/self.myNet.turnings(self.id, outRoads(k)));
                    end
                end
                
                if minimum == 1e10
                    f = min( self.getDemand, Sout(self.id, time));
                else
                    f = min (minimum(:), self.getDemand);
                end
                self.computedOutFlow = f;
            end
        end
        
    end 
end

