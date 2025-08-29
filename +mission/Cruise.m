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

        function tbl = run(obj, tab)0.01
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
            tStart = tab.Time(end);
            eulers_Start = tab.Eulers(end, :);
            phi = eulers_Start(:, 3);
            psi = eulers_Start(:, 3);

            % Heading check for adding change in distance in +/- x_I
            pos_NED_Start = tab.Position_NED(end, :);
            x_NED_Start = pos_NED_Start(1); %
            if abs(abs(psi) - 180) < 1
                delta_NED = - obj.dDelta;
            else
                delta_NED = obj.dDelta;
            end
            x_NED_End   = x_NED_Start + delta_NED;

            % Initialize table variables
            t = tStart*ones(numVals_, 1);
            x_NED = linspace(x_NED_Start, x_NED_End, numVals_); % Discretization Base
            pos_NED(:, 1) = x_NED;

            vAir_NED = vAir_NED_Start*ones(numVals_,1);
            v_NED = zeros(numVals_,1);
            thrust=zeros(numVals_, 1);
            q=zeros(numVals_, 1);
            CL=zeros(numVals_, 1);
            CD=zeros(numVals_, 1);
            mass =(tab.mass(end));                      % Constant
            dt= zeros(numVals_,1); % necessary?
            pUse = zeros(numVals_,1);
            aoa = zeros(numVals_,1);
        
            % [F_x_Body, F_z_Body] = Vehicle.reconcile_L_D(L, D, alpha)
        for i = 1:numVals_
            if i == 1
                dt = 0;
            else
                v_NED(i)  = (d(i)-d(i-1))/dt(i);
                t(i) = t(i-1) + dt(i-1);
                dt = t(i) - t(i-1);
            end
            v_NED(i) = v_NED(i) - vAir_NED; % Check adjustment
            aoa(i) = 0; % What we need to stay in flight
            q(i)  = 0.5*rho*v(i)^2; % 
            L  = mass*g;
            CL(i) = L / (Sref * q(i));
            CD(i) = CD0 + k*CL(i)^2;
            D  = L/(CL(i)/CD(i));
            [pUse(i), thrust(i)] = ThrustBackCalculated(v, obj.T_set); % Needs vx body
            a = (thrust(i) - D(i))/m; % Body
            % Body acceleration to NED accel
            % Integrate NED accel to NED vel

            a_Body = 0; % sum of forces
            vbar = 0; % Make this start at vStart and become the average of current and previous
            dt = (pos_NED(i, 1:2) - pos_NED(i, 1:2)) / vbar;
            % update t = tStart and update with previous dt after i = 1
            % update v = last V + dt*a
            % Get thrust
            % update a = sum of forces / mass
        end

        % Vectors: position, airspeed, groundspeed, eulers
        % New structure/table. All vectors in NED
            Snew.Airspeed_NED = vAir;
            Snew.Groundspeed_NED = vGround;
            Snew.Eulers = eulers;
            Snew.Position_NED = pos;
            Snew.Mass = mass*ones(numVals_,1);
            Snew.Thrust_Body = thrust;
            Snew.q = q;
            Snew.CL = CL;
            Snew.CD = CD;
            Snew.Alpha = alpha;
            Snew.Energy = E;
            Snew.Power = power;
            Snew.Time = t;
            Tnew = struct2table(Snew); % Make it a structure
            tbl = [tab; Tnew]; % Concatenate tables and return

        end
        
    end
end

