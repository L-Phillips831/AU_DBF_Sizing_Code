classdef Aircraft
    properties
        DragPolar (1,2) double
        Propulsion
        MTOW (1,1) double
    end
    methods
        function ac = aircaft(DragPolar,Propulsion,MTOW)
            ac.DragPolar = DragPolar;
            ac.Propulsion = Propulsion;
            ac.MTOW = MTOW;
        end
         
    end
end