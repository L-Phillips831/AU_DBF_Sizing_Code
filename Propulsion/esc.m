classdef esc
    properties
        maxContCurrent (1,1) double
        maxBurstCurrent (1,1) double
        efficiency (1,1) double = 0.99
    end
    methods
        function esc = esc(maxContCurrent,maxBurstCurrent)
            esc.maxContCurrent = maxContCurrent;
            esc.maxBurstCurrent = maxBurstCurrent;
        end
    end
end