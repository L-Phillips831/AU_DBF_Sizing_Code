classdef Propulsion
    properties
        battery (1,:) battery
        esc esc
        motor motor
        propeller propeller
    end
    methods
        function Prop = Propulsion(battery,esc,motor,propeller)
            Prop.battery = battery;
            Prop.esc = esc;
            Prop.motor = motor;
            Prop.propeller = propeller;
            Prop.motor = Prop.motor.calcMaxPower(Prop.battery);
        end
    end

    methods (Static)
        
        function RPM = calcRPM(Prop,ThrottleSetting)
            RPM = Prop.motor.Kv .* Prop.battery.voltage .* ThrottleSetting;
        end
    end
end