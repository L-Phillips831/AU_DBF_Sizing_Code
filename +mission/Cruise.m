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
            numVals_ = numVals;
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
        % - add ground and airspeed
        % - add aoa
        % - add fpa

        % General variables
            % tab  = obj.MissionTable;                                             
            Sref = obj.vehicle.Sref;     
            k    = obj.vehicle.k;            
            CD0  = obj.vehicle.CD0;          
            rho  = obj.vehicle.rho;      
            g    = 9.81;    % Acceleration due to gravity [m/s^2]
            numVals_ = obj.numVals;

            vAir_NED_Start = tab.Airspeed_NED(end, :);
            v_NED_Start = tab.Groundspeed_NED(end, :);
            vWind_NED_Start = tab.Windspeed_NED(end, :);
            tStart = tab.Time(end);
            alpha_Start = tab.Alpha(end);
            eulers_Start = tab.Eulers(end, :);
            phi = eulers_Start(:, 1);
            theta = eulers_Start(:, 2);
            psi = eulers_Start(:, 3);

            % Initialize table variables
            t = tStart*ones(numVals_, 1);
            vAir_NED = vAir_NED_Start*ones(numVals_,1);
            v_NED = zeros(numVals_,1);
            vWind_NED = vWind_NED_Start*ones(numVals_,1);
            thrust=zeros(numVals_, 1);
            q=zeros(numVals_, 1);
            CL=zeros(numVals_, 1);
            CD=zeros(numVals_, 1);
            mass =(tab.mass(end));                   
            pUse = zeros(numVals_,1);
            alpha = zeros(numVals_,1);
            gamma = zeros(numVals_,1);

            % Heading check for adding change in distance in +/- x_I
            pos_NED_Start = tab.Position_NED(end, :);
            x_NED_Start = pos_NED_Start(1); %
            if abs(abs(psi) - 180) < 1
                delta_NED = - obj.dDelta;
            else
                delta_NED = obj.dDelta;
            end
            x_NED_End   = x_NED_Start + delta_NED;

            x_NED = linspace(x_NED_Start, x_NED_End, numVals_); % Discretization Base
            pos_NED(:, 1) = x_NED;
        
            % 

            % Flow:
            % Use v_NED and vWind_NED to get vAir_NED
            % Back calculate aoa from lift required
            % Get q, CL, CD, D to track or use later
            % Do a force balance in body frame with fn, thrust, and weight
            % Find acceleration in each direction
            % Numerically integrate acceleration


        for i = 1:numVals_
            weight = mass * g;
            vAir_NED(i, :) = v_NED(i, :) - vWind_NED(i, :); % Use previous v_NED

            if i == 1
                dt = 0;
                vAir_x_Body = cosd(alpha_Start) * vAir_NED(i, 1); % Need X velocity for thrust calcs
                [pUse(i), thrust(i)] = ThrustBackCalculated(vAir_x_Body, obj.T_set);
                L = weight - sind(alpha_Start) * thrust(i); % Use previous alpha as an approximation
            else
                v0 = abs(v_NED(i, 1));
                a = abs(a_NED(1));
                d = abs(pos_NED(i, 1) - pos_NED(i-1, 1));
                dt = (-v0 + sqrt(v0^2 - 2*a*d))/a; % Solved quadratic eqn
                t(i) = t(i-1) + dt;
                vAir_x_Body = cosd(alpha(i-1)) * vAir_NED(i, 1);
                [pUse(i), thrust(i)] = ThrustBackCalculated(vAir_x_Body, obj.T_set); % NEED FUNCTION
                L = weight - sind(alpha(i-1)) * thrust(i);
            end
            q(i) = 0.5 * rho * vAir_NED(i, 1)^2; % Only moves in x_NED
            CL(i) = L / (q(i) * Sref);
            alpha(i) = (CL(i) - CL0) / CL_alpha; % THESE AERO VALUES ARE IN VEHICLE?
            CD(i) = CD0 + k * CL(i)^2;
            D = CD(i) * q(i) * Sref;

            %[F_x_Body, F_z_Body] = Vehicle.reconcile_L_D(L, D, alpha); %
            %Use this in another function ^ ?

            
            F_x_NED = cosd(alpha(i)) * thrust(i) - D;
            a_NED = [F_x_NED, 0, 0]/mass;

            if i < numVals_
                v_NED(i+1) = v_NED(i) + a_NED*dt; % Next time step velocity
            end








        end

        % Vectors: position, airspeed, groundspeed, eulers
        % New structure/table. All vectors in NED
            Snew.Airspeed_NED = vAir_NED;
            Snew.Groundspeed_NED = vGround_NED;
            Snew.Windspeed_NED = vWind_NED;
            Snew.Eulers = eulers;
            Snew.Position_NED = pos_NED;
            Snew.Mass = mass*ones(numVals_,1);
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

