classdef Climb < mission.Mission_Segment
    %CLIMB Summary of this class goes here
    %   Detailed explanation goes here

%{
To do:
- down is positive
%}
    
    properties
        hEnd (1,1) double
        vClimb (1,1) double
        hDotClimb (1,1) double
        vehicle (1,1) Vehicle
        numVals (1,1) double

    end
    
    methods
        
        % Default constructor for the Cruise class.
        % Params:
        %   - hEnd: final height of the climb
        %   - vClimb: climb airspeed
        %   - hDotClimb: climb rate
        %   - vehicle: vehicle class with solved sizing
        %
        function obj = Climb(hEnd, vClimb, hDotClimb, numVals, vehicle)
            obj.hEnd = hEnd;
            obj.vClimb = vClimb;
            obj.hDotClimb = hDotClimb;
            obj.vehicle = vehicle;
            obj.numVals = numVals;

        end




        function tbl = run(obj, tab)
        % Discretize over altitude from hStart to hEnd.
        % Find flight path angle from climb velocity and vertical velocity,
        % which are both constants as inputs.  Assume thrust, body x
        % direction, and velocity in the same direction.


        % General variables
            Sref     = obj.vehicle.Sref;     
            k        = obj.vehicle.k;            
            CD0      = obj.vehicle.CD0;
            CL0      = obj.vehicle.CL0;
            CL_alpha = obj.vehicle.CL_alpha;
            rho      = obj.vehicle.rho;      
            g        = 9.81;    % Acceleration due to gravity [m/s^2]
            numVals_ = obj.numVals;
            v_Set    = obj.v_Climb;

            % Initialize table variables
            h_Start = tab.Altitude(end);
            h_End = obj.hEnd;
            tStart  = tab.Time(end);
            E_Start  = tab.Energy(end);
            vAir_NED_Start = tab.Airspeed_NED(end, :);
            v_NED_Start = tab.Groundspeed(end, :);
            vWind_NED_Start = tab.Windspeed_NED(end, :);
            pos_NED_Start = tab.Position_NED(end, :);

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
            E         = E_Start*ones(numVals_, 1);
            alpha     = zeros(numVals_,1);
            eulers    = repmat(eulers_Start, numVals_, 1);
            gamma     = gamma_Set*ones(numVals_,1);  

            % Preliminary euler setup
            if abs(abs(psi_Start) - 180) < 1
                direc_Scalar = -1;
            else
                direc_Scalar = 1;
            end

            % Prior calcs:
            weight = mass * g;
            d_Alt = h(2) - h(1);
            pos_NED(:, 3) = linspace(h_Start, h_End, numVals_);

            for i = 1:numVals_
                eulers(i, 2) = alpha(i) + gamma(i);
                [power(i), thrust(i)] = obj.Propulsion.get_Thrust(obj.T_set, vAir_x_Body);
                q(i) = 0.5 * rho * v_Set^2;
                CL(i) = weight / (q(i) * Sref);
                CD(i) = CD0 + k*CL(i);
                alpha(i) = (CL(i) - CL0) / CL_alpha;

                drag = CD(i) * Sref * q(i);
                pSpec = (thrust(i) - drag) * v_Set / weight;
                vAir_NED(i, 3) = pSpec;
                vAir_NED(i, 1) = direc_Scalar*sqrt(v_Set^2 - pSpec^2); % direction
                v_NED(i, 3) = pSpec;
                v_NED(i, :) = vWind_NED(i, :) + vAir_NED(i, :);
                gamma(i) = atand(v_NED(i, 3) / v_NED(i, 1));

                if i > 1
                    dt = d_Alt / v_NED(i, 2);
                    t(i) = t(i-1) + dt;
                end
                pos_NED(i, :) = pos_NED_Start + trapz(t(1:i), v_NED);
                E(i) = E_Start + trapz(t(1:i), power(1:i));
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

