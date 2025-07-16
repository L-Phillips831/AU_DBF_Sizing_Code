classdef wiring
    properties
        length (1,1) double % wire length (m)
        resistivity (1,1) double = 1.68e-8 % wire resistivity (dependant on wire material)
        gauge (1,1) double % wire gauge (american)
        Rm (1,1) double % wire resistance {Ohm}
    end
    methods
        function wire = wiring(length, gauge)
            wire.length = length;
            wire.gauge = gauge;
            area = wire.calcWireArea;

            wire.Rm = wire.resistivity * wire.length / area;
        end

        function area = calcWireArea(wire)
            % calculates wire area in m^2 from gauge
            area = 0.012668*92^((36-wire.gauge)/19.5) * 1e-6;
        end
    end
end