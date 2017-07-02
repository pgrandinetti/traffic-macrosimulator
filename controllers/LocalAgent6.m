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

classdef LocalAgent6 < handle
   % with (primalSdpU - u(k))^2 in the cost
    
    properties (SetAccess = public)
        primalVarY; % only yi^i
        %primalVarYLabels;
        primalVarU;
        primalVarULabels;
        %dualVarYLocal;
        %dualVarYNeig;
        dualVarULocal;
        dualVarUNeig;
        dualVarULabels;
        
        isInitialized; % true if variables have been initialized
        isConverged;
        
        myId;
        neighUp; neighDown; kinDown;
        
        % primal variables of the neighborhood for dual update
        % these will contain the variables collected during the algorithm.
        %yLocal;
        %yNeig;
        uLocal;
        uNeig;
        
        eps;
    end

    methods
        
        function L = LocalAgent6(id, net)
            L.myId = id;
            L.neighUp = net.neighborsIn(id);
            L.neighDown = net.neighborsOut(id);
            L.kinDown = net.downKin(id);
            
            % IMPORTANT: VARIABLE ORDER
            % y = first local y_i^i, then according to neighUp and then neighDown
            % u = first local u_i^i, then neighUp, then neighDown, then kinDown
            L.primalVarY = 0;
            %L.primalVarYLabels = [L.myId L.neighUp L.neighDown];
            L.primalVarU = zeros(1+length(L.neighUp)+length(L.neighDown)+length(L.kinDown),1);
            L.primalVarULabels = [L.myId L.neighUp L.neighDown L.kinDown];
            
            % store lambda_i^(i,j) separately from lambda_j^(i,j)
            %L.dualVarYLocal = zeros(length(L.neighUp)+length(L.neighDown),1);
            %L.dualVarYNeig = zeros(length(L.neighUp)+length(L.neighDown),1);
            % thus in principle the labels would be dualVarYLabels = neighDown
            
            % store separately also for lambda_u
            L.dualVarULocal = zeros(length(L.neighUp)+length(L.neighDown)+length(L.kinDown), 1);
            L.dualVarUNeig = zeros(length(L.neighUp)+length(L.neighDown)+length(L.kinDown), 1);
            L.dualVarULabels = [L.neighUp L.neighDown L.kinDown];
            
            % isInitialized says whether the variables have been
            % initialized
            L.isInitialized = 1; % yes because zero is a valid initial condition
            L.isConverged = 0;
            
            % yLocal will contain yi^j 
            %L.yLocal = zeros(length(L.neighUp)+length(L.neighDown),1);
            % yNeig will contain yj^j
            %L.yNeig = zeros(length(L.neighUp)+length(L.neighDown),1);
            % uLocal will contain ui^p
            L.uLocal = zeros(length(L.neighUp)+length(L.neighDown)+length(L.kinDown),1);
            % uNeig will contain up^p
            L.uNeig = zeros(length(L.neighUp)+length(L.neighDown)+length(L.kinDown),1);
            
            L.eps = 0.01;
        end
        
        
        function var = transmitLocalVar(self, j, type)
            % someone is asking how does self see j-th var in its local problem
            % j is the global id of the requiring agent (i.e. given by the network)
 
            if type=='y'
%                 if j==self.myId
%                     var = self.primalVarY(1);
%                 else
%                     var = self.primalVarY(self.primalVarYLabels==j);
%                 end
            else
                if j==self.myId
                    var = self.primalVarU(1);
                else
                    var = self.primalVarU(self.primalVarULabels==j);
                end
            end  
        end
        
        
        function [] = collectNeigVar(self, agentsList)
            uLabels = [self.neighUp self.neighDown self.kinDown];
            % it needs to collect ui^p, up^p
%             for j = 1 : length(self.yLocal)
%                 keyboard
%                 self.yLocal(j) = agentsList(yLabels(j)).transmitLocalVar(self.myId, 'y');
%                 self.yNeig(j) = agentsList(yLabels(j)).transmitLocalVar(yLabels(j), 'y');
%             end
            for p = 1 : length(self.uLocal)
                self.uLocal(p) = agentsList(uLabels(p)).transmitLocalVar(self.myId, 'u');
                self.uNeig(p) = agentsList(uLabels(p)).transmitLocalVar(uLabels(p), 'u');
            end
            
            % check convergence
            self.isConverged = 1;
            for p = 1 : length(self.uLocal)
                if abs(self.uLocal(p)-self.primalVarU(1)) > self.eps
                    self.isConverged = 0;
                    break
                end
            end
        end
        
        
        function [] = dualUpdate(self, alpha)
            % apply gradient descent on 
            % dualVarULocal, dualVarUNeig
            
            % uLocal, uNeig are variables previously
            % collected with collectNeigVar()
            
            % lambdaiy^(i,j)
%             for j = 1 : length(self.dualVarYLocal)
%                 self.dualVarYLocal(j) = self.dualVarYLocal(j) +...
%                     alpha* (self.primalVarY(1) - self.yLocal(j));
%             end
%             
%             % lambdajy^(i,j)
%             for j = 1 : length(self.dualVarYNeig)
%                self.dualVarNeig(j) = self.dualVarNeig(j) + ...
%                    alpha* (self.primalVarY(j) - self.yNeig(j));
%             end
            
            % lambdaiu^(i,p)
            for p = 1 : length(self.dualVarULocal)
                self.dualVarULocal(p) = self.dualVarULocal(p) + ...
                    alpha* (self.primalVarU(1) - self.uLocal(p));
            end
            
            % lambdapu^(i,p)
            for p = 1 : length(self.dualVarUNeig)
                self.dualVarUNeig(p) = self.dualVarUNeig(p) + ...
                    alpha* (self.primalVarU(p+1) - self.uNeig(p));
            end            
            
        end
        
        
        function []  = primalUpdate(self, net, Din, Sout, k)
            %disp(strcat(['Agent ',int2str(self.myId)]));
            
            % k is the current time instant
            % the primal update is a minimization of the
            % lagrangian
            
            % IMPORTANT: PROBLEM VARIABLES HAVE THE SAME ORDER OF THE STATE VARIABLES
            primalSdpY = sdpvar(1);
            %primalVarYLabels = [i neighDown];
            primalSdpU = sdpvar(1+length(self.neighUp)+length(self.neighDown)+length(self.kinDown),1);
            
            % initialize the lagrangian
            % STEP 1
            % build matrix H
            H = buildHMatrix(self, net, Din, Sout, k);
            
            % STEP 2
            % build balancing measure
            [bal, model] = buildBal(self, net, H, primalSdpU);
            
            % CUMULATE BALANCING E TTD (PRIMAL OBJECTIVE) IN THE LAGRANGIAN
            factorBal = 1;
            factorTTD = 1/net.roads(self.myId).maxFlow;
            L = factorBal* bal - factorTTD* primalSdpY;
            %keyboard
            % STEP 3
            % build the relaxed constraints
            
%                         % 3.1 part for y
%             
%                         for j = 1 : length(self.neighDown)
%             
%                             % there are two terms, one with yi^i and another with yj^i
%             
%                             % first term is yi^i * 2 lambdai^(i,j)
%                             L = L + primalSdpY(1) * 2 * self.dualVarYLocal(j);
%             
%                             % second term is yj^i * 2 lambdaj^(i,j)
%                             L = L + primalSdpY(1+j) * 2 * self.dualVarYNeig(j);
%             
%                         end
            
            % 3.2 part for u
            %keyboard
            factorDual = 1;
            for p = 1 : length(self.dualVarULabels)
                % first term is ui^i * 2 lambdai^(i,p)
                %L = L + factorDual* primalSdpU(1) * 2 * self.dualVarULocal(p);
                L = L + factorDual* self.dualVarULocal(p)* (primalSdpU(1) - self.uLocal(p));
                % second term is up^i * 2 lambdap^(i,p)
                %L = L + factorDual* primalSdpU(p+1) * 2 * self.dualVarUNeig(p);
                L = L + factorDual* self.dualVarUNeig(p)* (primalSdpU(p+1)-self.uNeig(p));
            end
            
            % STEP 4
            % add local constraints that are not dualized
            %keyboard
            F = [model, primalSdpU>=0.1, primalSdpU<=1, primalSdpY>=0, primalSdpY<=net.roads(self.myId).maxFlow];
            F = [F, sum([primalSdpU(1); primalSdpU(ismember(self.primalVarULabels,self.kinDown))])<=1,...
                sum(primalSdpU(ismember(self.primalVarULabels,self.neighUp)))<=1];
            
            if isempty(self.neighUp)
                % use the trick with H
                tmp = net.roads(self.myId).currentDensity + ...
                    H{1}(1) + H{1}(end)*primalSdpU(1);
            else
                order = arrayfun(@(x) find(self.primalVarULabels==x), self.neighUp);
                tmp = net.roads(self.myId).currentDensity +...
                    dot(H{1}, [primalSdpU(order); primalSdpU(ismember(self.primalVarULabels, self.myId))]);
            end
            
            F = [F, primalSdpY <= net.roads(self.myId).maxSpeed * tmp,...
                primalSdpY <= net.roads(self.myId).congSpeed*(net.roads(self.myId).maxDensity-tmp)];
            
            % STEP 5
            % add smooth term for u
            %keyboard
            duty0 = net.getDutyCycles();
            duty0 = duty0(self.primalVarULabels);
            for j = 1 : length(duty0)
                div = 1+length(net.neighborsIn(self.primalVarULabels(j)))+...
                    length(net.neighborsOut(self.primalVarULabels(j)))+...
                    length(net.downKin(self.primalVarULabels(j)));
                L = L+ (primalSdpU(j)-duty0(j))^2 / div;
            end
            %
            
%             % build var y for neighDown
%             %keyboard
%             if ~isempty(self.neighDown)
%                 yVar = sdpvar(length(self.neighDown),1);
%                 F = [F, yVar>=0];
%                 
%                 for i = 1 : length(self.neighDown)
%                     iNeighUp = net.neighborsIn(self.neighDown(i));
%                     order = arrayfun(@(x) find(self.primalVarULabels==x), iNeighUp);
%                     tmp = net.roads(self.neighDown(i)).currentDensity +...
%                         dot(H{i+1}, [primalSdpU(order); primalSdpU(ismember(self.primalVarULabels,self.neighDown(i)))]);
%                     F = [F, yVar(i)<= net.roads(self.neighDown(i)).maxFlow, ...
%                         yVar(i) <= net.roads(self.neighDown(i)).maxSpeed * tmp, ...
%                         yVar(i) <= net.roads(self.neighDown(i)).congSpeed*(net.roads(self.neighDown(i)).maxDensity-tmp)];
%                 end
%                 
%                 L = L - sum(yVar./[net.roads(self.neighDown).maxFlow]')/(1+length(self.kinDown));
%                 %L = L - sum(yVar)/(1+length(self.kinDown));
%             end
%             
%                         for i = 1 : length(primalSdpY)
%                             if isempty(net.neighborsIn(self.primalVarYLabels(i))) % this may happen only for self
%                                 % use the trick with H
%                                 tmp = net.roads(self.primalVarYLabels(i)).currentDensity +...
%                                     H{i}(1) + H{i}(end)*primalSdpU(self.primalVarULabels(i));
%                             else
%                                 iNeighUp = net.neighborsIn(self.primalVarYLabels(i));
%                                 order = arrayfun(@(x) find(self.primalVarULabels==x), iNeighUp);
%                                 tmp = net.roads(self.primalVarYLabels(i)).currentDensity +...
%                                     dot(H{i}, [primalSdpU(order); primalSdpU(ismember(self.primalVarULabels,self.primalVarYLabels(i)))]);
%                             end
%                             F = [F, primalSdpY(i) <= net.roads(self.primalVarYLabels(i)).maxSpeed * tmp, ...
%                                 primalSdpY(i) <= net.roads(self.primalVarYLabels(i)).congSpeed*(net.roads(self.primalVarYLabels(i)).maxDensity-tmp)];
%                         end
            
            % now optimize
            optimize(F, L, sdpsettings('solver', 'mosek', 'verbose', 0));
            
            % check convergence
            %             if ( sum(abs(self.primalVarY - value(primalSdpY)) <= self.eps) == length(self.primalVarY) && ...
            %                    sum(abs(self.primalVarU - value(primalSdpU)) <= self.eps) == length(self.primalVarU) )
            %                self.isConverged = 1;
            %             end
%             if ( sum(abs(self.primalVarU - value(primalSdpU)) <= self.eps) == length(self.primalVarU) )
%                 self.isConverged = 1;
%             end

            % update primal Vars
            self.primalVarY = value(primalSdpY);
            self.primalVarU = value(primalSdpU);
            %disp(strcat(['Agent ', int2str(self.myId),' ', num2str(self.primalVarU')])); 
        end

        function H = buildHMatrix(self, net, Din, Sout, k)
            % save them in a cell because they may have different dimensions
            H = cell(1+length(self.neighDown),1);
            
            % First row is h_i
            % other rows are ordered as the result of net.neighborsDown(i)
            
            % 1.1
            % build vector h_j
            % every vecor is h_j = [ {beta_vj * outflow(v)}_v=neighUp(j), -outflow(j) ]
            for l=1 : length(H)
                
                % for what agent are we computing the vector h?
                if l==1
                    idAgent = self.myId;
                else
                    idAgent = self.neighDown(l-1);
                end
                
                % save entering road in this agent
                tmpNeighUp = net.neighborsIn(idAgent);
                
                % create the cell for it
                % IMPORTANT: WHEN THERE ARE NO neighUP WE ADD A CONSTANT
                % TERM GIVEN BY Din
                if ~isempty(tmpNeighUp)
                    H{l} = zeros(length(tmpNeighUp)+1,1);
                else 
                    H{l} = zeros(2,1); % to add the affine term
                end
                
                % populate cell
                for n = 1 : length(tmpNeighUp)
                    H{l}(n) = net.sampleTime/net.roads(idAgent).L*...
                        (net.turnings(tmpNeighUp(n),idAgent) * net.roads(tmpNeighUp(n)).ownOutFlow(Sout,k));
                end
                
                % if it was empty add the item given by Din
                if isempty(tmpNeighUp)
                    H{l}(1) = net.sampleTime/net.roads(idAgent).L*...
                        (min ( [Din(idAgent),...
                        net.roads(idAgent).congSpeed * (net.roads(idAgent).maxDensity - net.roads(idAgent).currentDensity),...
                        net.roads(idAgent).maxFlow]));
                end

                % add last item
                H{l}(end) = net.sampleTime/net.roads(idAgent).L* (-net.roads(idAgent).ownOutFlow(Sout,k));     
            end
        end
        
        
        function [bal, Model] =  buildBal(self, net, H, primalSdpU)
            % H is a cell of vectors hi, with h_self in 1st position H{1}
            Model = [];
            bal = 0;
            
            % add all terms hi * u[i,j in neighUp(i)]
            
            % first build the local term
            if ~isempty(self.neighUp)
                localAheadDensity = dot(H{1}, [primalSdpU(2:1+length(self.neighUp)); primalSdpU(1)]) + ...
                    net.roads(self.myId).currentDensity;
            else
                localAheadDensity = net.roads(self.myId).currentDensity + H{1}(1) + H{1}(end).*primalSdpU(1);
            end
            
            %asd = sdpvar(length(self.neighDown),1);
            %e = sdpvar(length(self.neighDown),1);
            for j = 1 : length(self.neighDown)
                
                % build the term for the current j
                
                % the u variables for this term will be [ui (because neighUp of j), uj (itself), and
                % uq (with q kinDown of i = neighUp of j) ]
                
                jNeighUp = net.neighborsIn(self.neighDown(j));
                
                % reorder variables u as needed from j
                order = arrayfun(@(x) find(self.primalVarULabels==x), jNeighUp);
                %keyboard
                % add to balancing
                asd = dot(H{j+1}, [primalSdpU(order); primalSdpU(self.primalVarULabels == self.neighDown(j))]) + ...
                    net.roads(self.neighDown(j)).currentDensity;
                %bal = bal + ((localAheadDensity - asd)/net.roads(self.myId).maxDensity)^2;
                bal = bal + ((localAheadDensity - asd))^2;
                %bal = bal + ((localAheadDensity - asd)^2)/(net.roads(self.myId).maxDensity);
                
            end

            bal = bal/net.roads(self.myId).maxDensity;
            
            %Model = [e == localAheadDensity - asd];
            %bal = dot(e,e);
        end
    end
end
