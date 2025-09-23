classdef Climb < mission.Mission_Segment
    %CLIMB Summary of this class goes here
    %   Detailed explanation goes here

%{
To do:
- down is positive
%}
    
    properties
        hEnd (1,1) double
        T_set (1,1) double
        vehicle Vehicle
        numVals (1,1) double

    end
    
    methods
        % Default Constructor Method
        function obj = Climb(hEnd, T_set, numVals, vehicle)
            obj.hEnd = hEnd;
            obj.T_set = T_set;
            obj.vehicle = vehicle;
            obj.numVals = numVals;

        end




        function tbl = run(obj, tab)
        % Discretize over altitude from hStart to hEnd.


        % General variables
            Sref     = obj.vehicle.S_ref;     
            k2        = obj.vehicle.K2;
            k1       = obj.vehicle.K1;
            Cd_0     = obj.vehicle.CD_0;
            rho      = obj.vehicle.rho;      
            g        = 9.81;    % Acceleration due to gravity [m/s^2]
            numVals_ = obj.numVals;
            vClimb_    = norm(tab.Airspeed_NED(end, :));

            % Initialize table variables
            h_End           = obj.hEnd;
            tStart          = tab.Time(end);
            E_Start         = tab.Energy(end);
            vAir_NED_Start  = tab.Airspeed_NED(end, :);
            v_NED_Start     = tab.Groundspeed_NED(end, :);
            vWind_NED_Start = tab.Windspeed_NED(end, :);
            pos_NED_Start   = tab.Position_NED(end, :);
            eulers_Start    = tab.Eulers(end,:);

        % Initialize table variables
            t         = tStart*ones(numVals_, 1);
            vAir_NED  = repmat(vAir_NED_Start, numVals_, 1);
            v_NED     = repmat(v_NED_Start, numVals_, 1);
            vWind_NED = repmat(vWind_NED_Start, numVals_, 1);
            pos_NED   = repmat(pos_NED_Start, numVals_, 1);
            throttle  = obj.T_set*ones(numVals_, 1);
            thrust    = zeros(numVals_, 1);
            q         = zeros(numVals_, 1);
            CL        = zeros(numVals_, 1);
            CD        = zeros(numVals_, 1);
            mass      = (tab.Mass(end));                   
            power     = zeros(numVals_,1);
            E         = zeros(numVals_,1);
            alpha     = zeros(numVals_,1);
            eulers    = repmat(eulers_Start, numVals_, 1);
            gamma     = zeros(numVals_,1);  

            % Preliminary euler setup
            if abs(abs(eulers_Start(3)) - 180) < 1
                direc_Scalar = -1;
            else
                direc_Scalar = 1;
            end

            % Non-iterative calcs:
            weight = mass * g;
            Z_coords   = linspace(pos_NED_Start(3), -h_End, numVals_);
            d_Z  = Z_coords(2) - Z_coords(1);
            RPM = obj.vehicle.prop.calcRPM(obj.T_set);
            [thrust_use, power_use] = obj.vehicle.prop.get_Thrust_Power(RPM, vClimb_);
            q_use     = 0.5 * rho * vClimb_^2;
            CL_use    = weight / (q_use * Sref);
            alpha_use = obj.vehicle.get_req_alpha(CL_use);
            CD_use    = Cd_0 + k1*CL_use + k2*CL_use^2;
            drag      = CD_use * Sref * q_use;
            pSpec     = (thrust_use - drag) * vClimb_ / weight;
            if pSpec >= vClimb_/sqrt(2)
                pSpec = vClimb_/sqrt(2);
            end
            dt        = abs(d_Z / -pSpec);

        % Tracking
            for i = 1:numVals_
                vAir_NED(i, 3)  = -pSpec;
                vAir_NED(i, 1)  = direc_Scalar*sqrt(vClimb_^2 - pSpec^2); 
                v_NED(i, :)     = vAir_NED(i, :) + vWind_NED(i, :);
                alpha(i)        = alpha_use;
                gamma(i)        = atand(-v_NED(i, 3) / v_NED(i, 1));
                eulers(i, 2)    = alpha(i) + gamma(i);
                thrust(i)       = thrust_use;
                q(i)            = q_use;
                CL(i)           = CL_use;
                CD(i)           = CD_use;
                power(i)        = power_use;
                if i>1
                t(i) = t(i-1) + dt;
                end
                E(i)          = E_Start + (t(i) - t(1))*power_use;  
                if i>1
                pos_NED(i, 1) = pos_NED_Start(1) + trapz(t(1:i), v_NED(1:i, 1));
                pos_NED(i, 2) = pos_NED_Start(2) + trapz(t(1:i), v_NED(1:i, 2));
                pos_NED(i, 3) = Z_coords(i);
                end
            end

        % New structure/table. All vectors in NED
            Snew.Airspeed_NED    = vAir_NED;
            Snew.Groundspeed_NED = v_NED;
            Snew.Windspeed_NED   = vWind_NED;
            Snew.Eulers          = eulers;
            Snew.Position_NED    = pos_NED;
            Snew.Mass            = mass*ones(numVals_,1);
            Snew.Throttle        = throttle;
            Snew.Thrust_Body     = thrust;
            Snew.q               = q;
            Snew.CL              = CL;
            Snew.CD              = CD;
            Snew.Alpha           = alpha;
            Snew.Gamma           = gamma;
            Snew.Energy          = E;
            Snew.Power           = power;
            Snew.Time            = t;
            Tnew = struct2table(Snew); % Make it a structure
            tbl = [tab; Tnew]; % Concatenate tables and return
        end
    end
end

