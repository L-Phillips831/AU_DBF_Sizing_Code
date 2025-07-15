classdef battery
    properties
        capacity (1,1) double % Battery Capacity {mah}
        voltage (1,1) double % Battery Voltage {V}
        cellVoltage (1,1) double = 3.7
        cellCount (1,1) double {mustBeInteger} % Battery Cell Count
        CRating (1,1) double {mustBeInteger} % C-Rating
        maxCurrent (1,1) double % Maximum current draw mbattery can support {A}
        composition (1,1) string {mustBeMember(composition,"LiPo")} = "LiPo" % Battery Chemical Composition
        energyCapacity (1,1) double % Energy rating of battery {Wh}
        stateOfCharge (1,1) double  = 100 % state of Charge in %(0-100)
    end
    methods
        function bt = battery(capacity,cellCount,CRating,composition)
            bt.capacity = capacity;
            bt.cellCount = cellCount;
            bt.composition = composition;
            bt.CRating = CRating;
            bt.maxCurrent = bt.CRating * bt.capacity / 1000;
            
            switch bt.composition
                case "LiPo"
                    bt.cellVoltage = 3.7;
                    bt.voltage = bt.cellCount .* bt.cellVoltage;
            end
            bt.energyCapacity = bt.voltage .* bt.capacity / 1000;
        end
    end
end