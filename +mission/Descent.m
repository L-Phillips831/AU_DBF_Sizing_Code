classdef Descent < Mission_Segment
    %DESCENT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        hEnd (1,1) double
        vehicle (1,1) Vehicle
        numVals (1,1) double

    end
    
    methods
        
        % Default constructor for the Descent class.
        function obj = Climb(hEnd, numVals, vehicle)
            obj.hEnd = hEnd;
            obj.vehicle = vehicle;
            obj.numVals = numVals;
        end
        
        function tbl = run(obj, tab)
        % Discretize over altitude from starting Z position to hEnd.
        % Similar to climb segment method but flight speed is a simple
        % multiple of stall speed and angle of descent is dictated by
        % throttle setting

        % General variables
            Sref     = obj.vehicle.Sref;     
            k        = obj.vehicle.k;            
            Cd_0     = obj.vehicle.Cd_0;
            Cl_max   = obj.vehicle.Cl_max;
            rho      = obj.vehicle.rho;      
            g        = 9.81;    % Acceleration due to gravity [m/s^2]
            numVals_ = obj.numVals;
            k_safe   = 1.5;

            % Initialize table variables
            h_End           = obj.hEnd;
            tStart          = tab.Time(end);
            E_Start         = tab.Energy(end);
            vAir_NED_Start  = tab.Airspeed_NED(end, :);
            v_NED_Start     = tab.Groundspeed(end, :);
            vWind_NED_Start = tab.Windspeed_NED(end, :);
            pos_NED_Start   = tab.Position_NED(end, :);

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
            mass      = (tab.mass(end));                   
            power     = zeros(numVals_,1);
            E         = zeros(numVals_,1);
            alpha     = zeros(numVals_,1);
            eulers    = repmat(eulers_Start, numVals_, 1);
            gamma     = zeros(numVals_,1);  

            % Preliminary euler setup
            if abs(abs(psi_Start) - 180) < 1
                direc_Scalar = -1;
            else
                direc_Scalar = 1;
            end

            % Non-iterative calcs:
            weight   = mass * g;
            v_stall = sqrt(weight / (Cl_max * Sref * 0.5 * rho));
            v_descent = k_safe * v_stall;
            Z_coords = linspace(pos_NED_Start(3), -h_End, numVals_);
            d_Z      = Z_coords(2) - Z_coords(1);
            descent_flag = 0;
            while descent_flag ~= 1
                [thrust_use, power_use] = obj.Propulsion.get_Thrust_Power(obj.T_set, v_descent);
                q_use     = 0.5 * rho * v_descent^2;
                CL_use    = weight / (q_use * Sref);
                alpha_use = obj.vehicle.get_req_alpha(obj, CL_use);
                CD_use    = Cd_0 + k*CL_use^2;
                drag      = CD_use * Sref * q_use;
                pSpec     = (thrust_use - drag) * v_descent / weight;
                dt        = d_Z / -pSpec;
                if pSpec < 0
                    descent_flag = 1;
                else
                    T_set = T_set - 0.05;
                end
            end

        % Tracking
            for i = 1:numVals_
                vAir_NED(i, 3)  = -pSpec;
                vAir_NED(i, 1)  = direc_Scalar*sqrt(v_descent^2 - pSpec^2); 
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
                pos_NED(i, 1) = pos_NED_Start(1) + trapz(t(1:i), v_NED(1:i, 1));
                pos_NED(i, 2) = pos_NED_Start(2) + trapz(t(1:i), v_NED(1:i, 2));
                pos_NED(i, 3) = Z_coords(i);
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

