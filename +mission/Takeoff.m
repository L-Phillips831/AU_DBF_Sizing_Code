classdef Takeoff < Mission_Segment
    %TAKEOFF Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        table
    end
    
    methods
        function MissionTable = Takeoff(obj)
        %TAKEOFF Construct an instance of this class
        %{
        Talk with cole about this one
        - Will have ground run, rotation, transition
        - Rotation discretized over both vLOF and aoaLOF
        - transition discretized over aoaLOF to aoaClimb and vLOF to vClimb
        - Does not account for sloped runway
        - Need propulsion equations
        - Need CL calculation from aoa
        %}
        % General variables
            tab  = obj.MissionTable;                                        % Need call
            Sref = obj.Sref;                                                % Need call
            k    = obj.k;                                                   % Need call
            CD0  = obj.CD0;                                                 % Need call
            rho  = obj.rho;                                                 % Need call
            mass = tab.mass(end);                                           % Need call

            % define object variables and discretization bounds
            gFriction = 0; % Need value
            g = 0; % Need value                                             
            aoaStart  = 0; % Need value
            vRot      = 0; % Need value
            vLOF      = 0; % Need value
            aoaLOF    = 0; % Need value
            vClimb    = 0; % Need value
            aoaClimb  = 0; % Need value
            numVals   = 0; % Need value
            numParts  = 3;                                                  % Ground run, rotation, transition
            Throttle  = 0; % Need vector/value

            % Initialize table variables
            t = zeros(numParts*numVals, 1);
            d = zeros(numParts*numVals, 1);
            v = zeros(numParts*numVals, 1);
            a = zeros(numParts*numVals, 1);
            aoa=zeros(numParts*numVals, 1);
            pUse=zeros(numParts*numVals, 1);
            thrust=zeros(numParts*numVals, 1);
            q=zeros(numParts*numVals, 1);
            CL=zeros(numParts*numVals, 1);
            CD=zeros(numParts*numVals, 1);
            L=zeros(numParts*numVals, 1);
            D=zeros(numParts*numVals, 1);
            LD=zeros(numParts*numVals, 1);
            hDot=zeros(numParts*numVals, 1);
            rho=(tab.rho)*ones(numParts*numVals, 1);                        % Constant
            h = zeros(numParts*numVals, 1); 
            Energy = zeros(numVals, 1); %

            % Other initializations
            dt= zeros(numParts*numVals,1);

            % Ground run
            v(1:numVals)   = linspace(0, vRot, numVals);                    % Discretization for GR
            aoa(1:numVals) = aoaStart;                                      % Good
            h(1:numVals)   = zeros(numVals, 1);                             % Good
            for i = 1:numVals
                [thrust(i), pUse(i)] = ThrustCalc(v(i), T_set(i));          % Need function
                q(i)  = 0.5*rho(i)*v(i)^2;                                  % Good
                CL(i) = CalcCL(aoa(i));                                     % Need function
                CD(i) = CD0 + k*CL(i)^2;                                    % Good
                LD(i) = CL(i)/CD(i);                                        % Good
                L(i)  = CL*Sref*q(i);                                       % Good
                D(i)  = (L(i)/LD(i)) + ((mass*g-L(i))*gFriction);           % Good
                a = (thrust(i) - D(i))/m;                                   % Good
                if i == 1
                    dt(1) = 0;                                              % dt starts at 0
                    t(1)  = 0;                                              % Time starts at 0
                    d(1)  = 0;                                              % Distance starts at 0
                else
                    % dt is dv/a
                    dt(i)         = (v(i)-v(i-1))/(0.5*(a(i-1)+a(i)));      % Good
                    t(i)          = t(i-1) + dt(i);                         % Good
                    d(i)          = 0.5*(v(i-1)+v(i))*dt(i);                % Good
                    Energy(i) = Energy(i-1) + 0.5*(pUse(i-1)+pUse(i))*dt;   % Good
                end
            end

        % Rotation
            v(numVals+1 : numVals*2) = linspace(vRot, vLOF, numVals);       % Discretized from vRot to vLOF
            aoa(numVals+1 : numVals*2) = linspace(0, aoaLOF, numVals);      % Assume constant rotation rate?
            h(numVals+1 : numVals*2) = zeros(numVals, 1);                   % Good
            for i = numVals+1 : numVals*2
                [thrust(i), pUse(i)] = ThrustCalc(v(i), Throttle(i));       % Need function
                q(i)  = 0.5*rho(i)*v(i)^2;                                  % Good
                CL(i) = CalcCL(aoa(i));                                     % Need function. Changing CL.
                CD(i) = CD0 + k*CL(i)^2;                                    % Good
                LD(i) = CL(i)/CD(i);                                        % Good
                L(i)  = CL*Sref*q(i);                                       % Good
                D(i)  = (L(i)/LD(i)) + ((mass*g-L(i))*gFriction);           % Good
                a = (thrust(i) - D(i))/m;                                   % Good
                dt(i)         = (v(i)-v(i-1))/(0.5*(a(i-1)+a(i)));      % Good
                t(i)          = t(i-1) + dt(i);                         % Good
                d(i)          = 0.5*(v(i-1)+v(i))*dt(i);                % Good
                Energy(i) = Energy(i-1) + 0.5*(pUse(i-1)+pUse(i))*dt;   % Good
            end

        % Transition
            v(2*numVals+1 : numVals*3) = linspace(vRot, vLOF, numVals);       % Discretized from vRot to vLOF
            aoa(2*numVals+1 : numVals*3) = linspace(aoaLOF, aoaClimb, numVals);      % Assume constant rotation rate?
        for i = 1:numVals
            h(i) = integrateHDot;
            logicTRANS()
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
            MissionTable = [tab; Tnew]; % Concatenate tables
        end
    end
end

