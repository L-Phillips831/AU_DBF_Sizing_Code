classdef Cruise < mission.Mission_Segment
    %CRUISE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = private)
        T_set (1,1) double
        dDelta (1,1) double
        vehicle (1,1) Vehicle
        numVals (1,1) double
        windVec (3, 1) double
    end
    
    methods
        function obj = Cruise(dDelta, T_set, numVals, vehicle, windVec)
            obj.T_set = T_set;
            obj.dDelta = dDelta;
            obj.vehicle = vehicle;
            obj.numVals_ = numVals;
            obj.windVec = windVec;
           
        end

        function tbl = run(obj, tab)
        % Description: This function maps the time history for a fixed throttle setting
        % on a straight flight path.
        % Flow: Discretized over distance. Get time steps using
        % current velocity and distance step. Lift equals weight and thrust
        % is found using velocity and throttle setting. Acceleration at
        % each step found with (Thrust - Drag) / mass.

        % To do:
        % - Update euler angles with aoa, assuming level flight

        % General variables
            Sref     = obj.vehicle.Sref;     
            k        = obj.vehicle.k;            
            CD0      = obj.vehicle.CD0;
            CL0      = obj.vehicle.CL0;
            CL_alpha = obj.vehicle.CL_alpha;
            rho      = obj.vehicle.rho;      
            g        = 9.81;    % Acceleration due to gravity [m/s^2]
            numVals_ = obj.numVals;

        % Pull start of segment conditions
            vAir_NED_Start = tab.Airspeed_NED(end, :);
            v_NED_Start = tab.Groundspeed_NED(end, :);
            vWind_NED_Start = tab.Windspeed_NED(end, :);
            tStart = tab.Time(end);
            psi_Start = tab.Eulers(end, 3);
            E_Start = tab.Energy(end);

            % Initialize table variables
            t         = tStart*ones(numVals_, 1);
            pos_NED   = zeros(numVals_, 1);
            vAir_NED  = repmat(vAir_NED_Start, numVals_, 1); 
            v_NED     = repmat(v_NED_Start, numVals_, 1);
            vWind_NED = repmat(vWind_NED_Start, numVals_, 1); 
            throttle  = obj.T_set*ones(numVals_, 1);
            thrust    = zeros(numVals_, 1);
            q         = zeros(numVals_, 1);
            CL        = zeros(numVals_, 1);
            CD        = zeros(numVals_, 1);
            mass      = (tab.mass(end));                   
            power     = zeros(numVals_,1);
            E         = E_Start*ones(numVals_, 1);
            alpha     = zeros(numVals_,1);
            eulers    = zeros(numVals_,3);
            gamma     = zeros(numVals_,1);

            % Heading check for adding change in distance in +/- x_I
            pos_NED_Start = tab.Position_NED(end, :);
            x_NED_Start   = pos_NED_Start(1);
            if abs(abs(psi_Start) - 180) < 1
                delta_NED = - obj.dDelta;
                direc_Scalar = -1;
            else
                delta_NED = obj.dDelta;
                direc_Scalar = 1;
            end
            x_NED_End     = x_NED_Start + delta_NED;
            x_NED         = linspace(x_NED_Start, x_NED_End, numVals_);     % Discretization Base
            pos_NED(:, 1) = x_NED;

            weight = mass * g;

        for i = 1:numVals_
            vAir_NED(i, :)        = v_NED(i, :) - vWind_NED(i, :);
            q(i)                  = 0.5 * rho * vAir_NED(i, 1)^2;
            alpha(i)              = ((weight/(q(i)*Sref)) - CL0) / CL_alpha;
            dt                    = abs(pos_NED(i, 1)) / v_NED(i, 1);
            vAir_x_Body           = cosd(alpha(i)) * vAir_NED(i, 1);
            [power(i), thrust(i)] = obj.vehicle.Propulsion.get_Thrust(obj.T_set, vAir_x_Body);
            L                     = weight - sind(alpha(i)) * thrust(i);
            CL(i)                 = L / (q(i) * Sref);
            eulers(i, :)          = [0, alpha(i), psi_Start];
            CD(i)                 = CD0 + k * CL(i)^2;
            D                     = CD(i) * q(i) * Sref;
            F_x_NED               = direc_Scalar * (cosd(alpha(i)) * thrust(i) - D); % Is this positive or negative x all the time?
            a_NED                 = [F_x_NED, 0, 0]/mass;
            if i < numVals_
                t(i) = t(i-1) + dt;
                v_NED(i+1) = v_NED(i) + a_NED*dt;
            end
            E(i) = E_Start + trapz(t(1:i) , power(1:i));
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

