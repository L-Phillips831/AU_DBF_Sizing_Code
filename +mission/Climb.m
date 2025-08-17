classdef Climb < Mission_Segment
    %CLIMB Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Property1
    end
    
    methods
        function [obj] = climb(obj, hEnd, vClimb, hDotClimb)
        % Discretize over altitude from hStart to hEnd.
        % Find flight path angle from climb velocity and vertical velocity,
        % which are both constants as inputs.  Assume thrust, body x
        % direction, and velocity in the same direction.


        % General variables
            tab  = obj.MissionTable;                                        % Need call
            Sref = obj.Sref;                                                % Need call
            k    = obj.k;                                                   % Need call
            CD0  = obj.CD0;                                                 % Need call
            rho  = obj.rho;                                                 % Need call
            mass = tab.mass(end);                                           % Need call
            g    = obj.g;                                                   % Need call

            % Initialize table variables
            hStart = tab.Altitude(end);
            dInit = tab.Distance(end);
            tInit  = tab.Time(end);
            Einit  = tab.Energy(end);

            % Make time history vectors
            h = linspace(hStart, hEnd, numVals);                            % Discretization parameter
            hDot = hDotClimb*ones(numVals, 1);
            vGround = sqrt(vClimb^2 - hDotClimb^2);
            fpa = atan2(hDotClimb, vGround(1));
            q = (0.5*rho*vClimb^2)*ones(numVals, 1);
            a = zeros(numVals, 1);
            L = mass*g*cos(fpa);
            CL = L ./ (Sref .* q);
            CD = CD0 + k*CL.^2;
            LD = CL ./ CD;
            D = L./LD;
            thrust = D + mass * g * sin(fpa);
            propPower = thrust * vClimb;
            pUse = PropCalc(propPower(1))*ones(numVals, 1);
            dt = (h(2)-h(1))/hDot*(0:numVals);
            d = dInit+vGround.*dt; % vec
            t = tInit + dt; % vec
            Energy = Einit + pUse*dt;

        % Form new structure/table and concatenate
            Snew.Time = t;
            Snew.E = Energy;
            Snew.Power = pUse;
            Snew.D = d;
            Snew.V_Ground = vGround;
            Snew.V = vClimb * ones(numVals, 1);
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

