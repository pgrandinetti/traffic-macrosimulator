% This file is part of Traffic MacroSimulator.
%
% Traffic MacroSimulator is free software: you can redistribute it and/or
% modify it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% You should have received a copy of the GNU General Public License
% along with Traffic MacroSimulator.  If not, see http://www.gnu.org/licenses/.

% @author: P. Grandinetti

% This function converts duty cycles to binary values
% according to the network status

function values = convertDutyCycles(net, duty)

    values = zeros(length(net.roads),net.period);
    for i = 1 : size(net.iM, 2)
        kin = net.kinIntersectionDown(i);
        limit = 1;
        for j = 1 : length(kin)
            v = round(net.period*duty(kin(j)));
            values(kin(j),limit:limit+v-1) = 1;
            limit = limit+v;
        end
    end
    out = net.exitingRoads();
    % this
    values(out,:) = ones(length(out),size(values,2));
    % or this.. ?
%     for i = 1 : length(out)
%          v = round(net.period*duty(out(i)));
%          values(out(i),:) = [ones(1,v) zeros(1,net.period-v)];
%     end
end

