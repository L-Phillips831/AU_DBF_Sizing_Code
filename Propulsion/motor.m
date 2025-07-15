classdef motor
    properties
        Kv (1,1) double % Motor Kv-rating {RPM/V}
        maxPower (1,1) double % Motor max power {W}
        maxCurrent (1,1) double % Motor max current {A}
        Rm (1,1) double = 0.013 % Motor Resistance {Ohms}
    end
    methods
        function mt = motor(Kv,maxCurrent,Rm)
            mt.Kv = Kv;
            mt.maxCurrent =maxCurrent;
            mt.Rm = Rm;
        end
        
        function mt = calcMaxPower(mt,bat)
            mt.maxPower = bat.voltage * mt.maxCurrent;
        end
    end
end