classdef Cruise < mission.Mission_Segment
    %CRUISE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = private)
        T_set (1,1) double
        dDelta (1,1) double
        vehicle (1,1) Vehicle
        numVals (1,1) double
    end
    
    methods
        function obj = Cruise(dDelta, T_set, numVals, vehicle)
            obj.T_set = T_set;
            obj.dDelta = dDelta;
            obj.vehicle = vehicle;
            obj.numVals = numVals;
           
        end

        function tbl = run(obj, tab)
        % Description: This function maps the time history for a fixed throttle setting
        % on a straight flight path.
        % Flow: Discretized over distance. Get time steps using
        % current velocity and distance step. Lift equals weight and thrust
        % is found using velocity and throttle setting. Acceleration at
        % each step found with (Thrust - Drag) / mass.

        % General variables
            % tab  = obj.MissionTable;                                             
            Sref = obj.vehicle.Sref;     
            k    = obj.vehicle.k;            
            CD0  = obj.vehicle.CD0;          
            rho  = obj.vehicle.rho;      
            g    = 9.81;    % Acceleration due to gravity [m/s^2]         

            vStart = tab.Velocity(end);
            dStart = tab.Distance(end);
            tStart = tab.Time(end);
            endD = dStart+ obj.dDelta;

            % Initialize table variables
            t = tStart*ones(obj.numVals, 1);
            d = linspace(dStart, endD, obj.numVals); % Discretization Base
            v = vStart*ones(obj.numVals,1);
            a=zeros(obj.numVals, 1);
            thrust=zeros(obj.numVals, 1);
            q=zeros(obj.numVals, 1);
            CL=zeros(obj.numVals, 1);
            CD=zeros(obj.numVals, 1);
            L=zeros(obj.numVals, 1);
            D=zeros(obj.numVals, 1);
            LD=zeros(obj.numVals, 1);
            mass=(tab.mass(end))*ones(obj.numVals, 1); % Constant
            hDot=zeros(obj.numVals); % Constant, 0
            h = (tab.alt(end))*ones(obj.numVals, 1); % Constant
            dt= zeros(obj.numVals,1);
            pUse = zeros(obj.numVals,1);
        
        for i = 1:obj.numVals
            if i == 1
                dt = 0;
            else
                v(i)  = (d(i)-d(i-1))/dt(i);
                dt = t(i) - t(i-1);
                t(i) = t(i-1) + dt(i-1);
            end
            q(i)  = 0.5*rho*v(i)^2;
            L(i)  = mass*g;
            CL(i) = L(i) / (Sref * q(i));
            CD(i) = CD0 + k*CL(i)^2;
            LD(i) = CL(i)/CD(i);
            D(i)  = L(i)/LD(i);
            [pUse(i), thrust(i)] = ThrustBackCalculated(v, obj.T_set);
            a = (thrust(i) - D(i))/m;
        end
        
        % New structure/table
            Snew.Time = t;
            Snew.E = Energy;
            Snew.Power = pUse;
            Snew.Distance = d;
            Snew.V = v;
            Snew.Acceleration = a;
            Snew.Altitude = h;
            Snew.hDot = hDot;
            Snew.Mass = mass*ones(obj.numVals,1);
            Snew.Thrust = thrust;
            Snew.q = q;
            Snew.CL = CL;
            Snew.CD = CD;
            Snew.Lift = L;
            Snew.Drag = D;
            Snew.LD = LD;
            Tnew = struct2table(Snew); % Make it a structure
            tbl = [tab; Tnew]; % Concatenate tables and return

        end
        
    end
end

