classdef Climb < mission.Mission_Segment
    %CLIMB Summary of this class goes here
    %   Detailed explanation goes here

%{
To do:
- numVals correction
- Thrust function correction
- for loop for time correction
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
            numVals_ = numVals;

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

            % Initialize table variables
            h_Start = tab.Altitude(end);
            h_End = obj.hEnd;
            tStart  = tab.Time(end);
            E_Start  = tab.Energy(end);

        % Initialize table variables
            t         = tStart*ones(numVals_, 1);
            vAir_NED  = repmat(vAir_NED_Start, numVals_, 1);
            v_NED     = repmat(v_NED_Start, numVals_, 1);
            vWind_NED = repmat(vWind_NED_Start, numVals_, 1);
            pos_NED   = pos_NED_Start*ones(numVals_, 3);
            throttle  = obj.T_set*ones(numVals_, 1);
            thrust    = zeros(numVals_, 1);
            q         = zeros(numVals_, 1);
            CL        = zeros(numVals_, 1);
            CD        = zeros(numVals_, 1);
            mass      = (tab.mass(end));                   
            power     = zeros(numVals_,1);
            E         = E_Start*ones(numVals_, 1);
            alpha     = zeros(numVals_,1);
            eulers  = repmat(eulers_Start, numVals_, 1);
            gamma     = zeros(numVals_,1);  

            % Preliminary euler setup
            if abs(abs(psi_Start) - 180) < 1
                direc_Scalar = -1;
            else
                direc_Scalar = 1;
            end

            % Prior calcs:
            weight = mass * g;
            d_Alt = h(2) - h(1);

            for i = 1:numVals_
                vAir_NED(i, :) = v_NED(i, :) - vWind_NED(i, :);
                v_NED
                eulers(i, 2) = alpha(i) + fpa(i);
                [power(i), thrust(i)] = obj.Propulsion.get_Thrust(obj.T_set, vAir_x_Body);
                vAero = sqrt(vAir_NED(i, 1)^2 + vAir_NED(i, 3)^2);
                q = 0.5 * rho * vAero^2;
                CL(i) = weight / (q(i) * Sref);
                CD(i) = CD0 + k*CL(i);
                alpha(i) = CL(i); % Does this track accurately?
                gamma(i) = atan2(v_NED(i, 1), v_NED(i, 2));
                if i > 1
                    dt = d_Alt / v_NED(i, 2);
                    t(i) = t(i-1) + dt;
                end
                pos_NED(i, :) = pos_NED_Start + trapz(t(1:i), v_NED);
                E(i) = E_Start + trapz(t(1:i), power(1:i));
            end

        % New structure/table. All vectors in NED
            Snew.Airspeed_NED = vAir_NED;
            Snew.Groundspeed_NED = v_NED;
            Snew.Windspeed_NED = vWind_NED;
            Snew.Eulers = eulers;
            Snew.Position_NED = pos_NED;
            Snew.Mass = mass*ones(numVals_,1);
            Snew.Throttle = throttle;
            Snew.Thrust_Body = thrust;
            Snew.q = q;
            Snew.CL = CL;
            Snew.CD = CD;
            Snew.Alpha = alpha;
            Snew.Gamma = gamma;
            Snew.Energy = E;
            Snew.Power = power;
            Snew.Time = t;
            Tnew = struct2table(Snew); % Make it a structure
            tbl = [tab; Tnew]; % Concatenate tables and return
        end
    end
end

