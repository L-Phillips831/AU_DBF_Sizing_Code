classdef esc
    properties
        maxContCurrent (1,1) double
        maxBurstCurrent (1,1) double
    end
    methods
        function esc = esc(maxContCurrent,maxBurstCurrent)
            esc.maxContCurrent = maxContCurrent;
            esc.maxBurstCurrent = maxBurstCurrent;
        end
    end
end