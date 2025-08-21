classdef Climb < mission.Mission_Segment
    %CLIMB Summary of this class goes here
    %   Detailed explanation goes here
    
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
            Sref = obj.vehicle.Sref;                                         
            k    = obj.vehicle.k;                                               
            CD0  = obj.vehicle.CD0;                                             
            rho  = obj.vehicle.rho;                                              
            mass = tab.mass(end);                                        
            g    = 9.81;    % Acceleration due to gravity [m/s^2]

            % Initialize table variables
            hStart = tab.Altitude(end);
            dInit = tab.Distance(end);
            tInit  = tab.Time(end);
            Einit  = tab.Energy(end);

            % Make time history vectors
            h = linspace(hStart, obj.hEnd, obj.numVals);                            % Discretization parameter
            hDot = obj.hDotClimb*ones(obj.numVals, 1);
            vGround = sqrt(obj.vClimb^2 - obj.hDotClimb^2);
            fpa = atan2(obj.hDotClimb, vGround(1));
            q = (0.5*rho*obj.vClimb^2)*ones(obj.numVals, 1);
            a = zeros(obj.numVals, 1);
            L = mass*g*cos(fpa);
            CL = L ./ (Sref .* q);
            CD = CD0 + k*CL.^2;
            LD = CL ./ CD;
            D = L./LD;
            thrust = D + mass * g * sin(fpa);
            propPower = thrust * obj.vClimb;
            pUse = PropCalc(propPower(1))*ones(obj.numVals, 1);
            dt = (h(2)-h(1))/hDot*(0:obj.numVals);
            d = dInit+vGround.*dt; % vec
            t = tInit + dt; % vec
            Energy = Einit + pUse*dt;

        % Form new structure/table and concatenate
            Snew.Time = t;
            Snew.E = Energy;
            Snew.Power = pUse;
            Snew.D = d;
            Snew.V_Ground = vGround;
            Snew.V = obj.vClimb * ones(obj.numVals, 1);
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

