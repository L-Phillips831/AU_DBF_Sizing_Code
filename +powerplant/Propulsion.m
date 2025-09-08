classdef Propulsion
    properties
        battery (1,:) powerplant.battery
        esc (1,:) powerplant.esc
        motor (1,:) powerplant.motor
        propeller (1,:) powerplant.propeller
        wire (1,:) powerplant.wiring
    end
    methods
        function prop = Propulsion(battery,esc,motor,propeller)
            prop.battery = battery;
            prop.esc = esc;
            prop.motor = motor;
            prop.propeller = propeller;
            % prop.wire = wire;
            prop.motor = prop.motor.calcMaxPower(prop.battery);
        end

        function RPM = calcRPM(prop, throttle_setting)

           RPM = prop.motor.Kv .* prop.battery.voltage .* throttle_setting;
     
        end

        function efficiency = calcMotorEfficiency(prop,powerOut)
            current = prop.battery.voltage ./ powerOut;
            powerIn = powerOut + (prop.motor.Io * prop.battery.voltage) + (prop.motor.Rm .* current.^2);

            efficiency = powerOut ./ powerIn;
        end

        function efficiency = calcWireEfficiency(prop,powerOut)
            current = prop.battery.voltage ./ powerOut;
            powerIn = powerOut + (prop.wire.Rm .* current.^2);

            efficiency = powerOut ./ powerIn;
        end

        function [Thrust, powerIn] = get_Thrust_Power(prop,RPM,V)
            powerOut = prop.propeller.GI_Power_fcn_V_RPM(V,RPM);

            powerOut(isnan(powerOut)) = 0;
            motorEfficiency = prop.calcMotorEfficiency(powerOut);

            powerIn = powerOut ./ (motorEfficiency + 1e-10);

            Thrust = prop.propeller.GI_Thrust_fcn_V_RPM(V,RPM);
        end
    end

    methods (Static)
        
    end
end