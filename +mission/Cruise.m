classdef Cruise < Mission_Segment
    %CRUISE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Property1
    end
    
    methods
        function obj = Cruise()
           
        end
        
        function [obj] = ConstantAccel(obj, dDelta)
        % Description: This function maps the time history for a fixed throttle setting
        % on a straight flight path.
        % Flow: Discretized over distance. Get time steps using
        % current velocity and distance step. Lift equals weight and thrust
        % is found using velocity and throttle setting. Acceleration at
        % each step found with (Thrust - Drag) / mass.

        % General variables
            tab  = obj.MissionTable;                                        % Need call
            Sref = obj.Sref;                                                % Need call
            k    = obj.k;                                                   % Need call
            CD0  = obj.CD0;                                                 % Need call
            rho  = obj.rho;                                                 % Need call
            g    = obj.g;                                                   % Need call        

            vStart = tab.Velocity(end);
            dStart = tab.Distance(end);
            tStart = tab.Time(end);
            endD = dStart+dDelta;
            numVals = obj.numVals;

            % Initialize table variables
            t = tStart*ones(numVals, 1);
            d = linspace(dStart, endD, numVals); % Discretization Base
            v = vStart*ones(numVals,1);
            a=zeros(numVals, 1);
            thrust=zeros(numVals, 1);
            q=zeros(numVals, 1);
            CL=zeros(numVals, 1);
            CD=zeros(numVals, 1);
            L=zeros(numVals, 1);
            D=zeros(numVals, 1);
            LD=zeros(numVals, 1);
            mass=(tab.mass(end))*ones(numVals, 1); % Constant
            hDot=zeros(numVals); % Constant, 0
            h = (tab.alt(end))*ones(numVals, 1); % Constant
            dt= zeros(numVals,1);
            pUse = zeros(numVals,1);
        
        for i = 1:numVals
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
            [pUse(i), thrust(i)] = ThrustBackCalculated(v, T_set);
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
            Snew.Mass = mass*ones(numVals,1);
            Snew.Thrust = thrust;
            Snew.q = q;
            Snew.CL = CL;
            Snew.CD = CD;
            Snew.Lift = L;
            Snew.Drag = D;
            Snew.LD = LD;
            Tnew = struct2table(Snew); % Make it a structure
            obj.MissionTable = [tab; Tnew]; % Concatenate tables
        end
    end
end

