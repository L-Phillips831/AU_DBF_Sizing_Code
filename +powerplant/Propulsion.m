classdef Propulsion
    properties
        battery (1,:) powerplant.battery
        esc powerplant.esc
        motor powerplant.motor
        propeller powerplant.propeller
    end
    methods
        function Prop = Propulsion(battery,esc,motor,propeller)
            Prop.battery = battery;
            Prop.esc = esc;
            Prop.motor = motor;
            Prop.propeller = propeller;
            Prop.motor = Prop.motor.calcMaxPower(Prop.battery);
        end

        function thrust = get_Thrust(Prop, throt_setting, airspeed)
            RPM = Propulsion.calcRPM(throt_setting);
            J = airspeed / ((RPM / 60) * Prop.propeller.diameter);

            thrust = Prop.propeller.GI_Thrust_fcn_J_RPM(J, RPM);

        end
    end

    methods (Static)
        
        function RPM = calcRPM(Prop,ThrottleSetting)
            RPM = Prop.motor.Kv .* Prop.battery.voltage .* ThrottleSetting;
        end
    end
end