classdef Takeoff < mission.Mission_Segment
    %TAKEOFF Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        table
    end
    
    methods
        function MissionTable = Takeoff(obj, vRot, vLOF, aoaLOF, vClimb, hDotClimb, throttle)
        % TAKEOFF Construct an instance of this class
        %{
        - Will have ground run, rotation, transition
        - Rotation discretized over both vLOF and aoaLOF
        - transition discretized over aoaLOF to aoaClimb and vLOF to vClimb
        - Does not account for sloped runway
        - Need propulsion equations
        - Need CL calculation from aoa
        %}


        % To Do:
        % - Distinguish ground velocity from velocity
        % - Put in the correct thrust/propulsion functions
        % - Add logic for transition


        % General variables
            tab  = obj.MissionTable;
            Sref = obj.Sref;
            k    = obj.k;
            CD0  = obj.CD0;
            rho  = obj.rho;
            g = obj.g;
            aoaSet = obj.aoaSet;
            grFriction = obj.grFriction;
            mass = tab.mass(end);
            numVals   = obj.numVals;

            numParts  = 3;                                                  % Ground run, rotation, transition

            % Initialize table variables
            t = zeros(numParts*numVals, 1);
            d = zeros(numParts*numVals, 1);
            vGround = zeros(numParts*numVals, 1);
            v = zeros(numParts*numVals, 1);
            a = zeros(numParts*numVals, 1);
            aoa=zeros((numParts-1)*numVals, 1);
            pUse=zeros(numParts*numVals, 1);
            thrust=zeros(numParts*numVals, 1);
            q=zeros(numParts*numVals, 1);
            CL=zeros(numParts*numVals, 1);
            CD=zeros(numParts*numVals, 1);
            L=zeros(numParts*numVals, 1);
            D=zeros(numParts*numVals, 1);
            LD=zeros(numParts*numVals, 1);
            hDot=zeros(numParts*numVals, 1);
            h = zeros(numParts*numVals, 1); 
            Energy = zeros(numParts*numVals, 1);


            % Other initializations
            dt= zeros(numParts*numVals,1);


            % Ground run
            vGround(1:numVals)   = linspace(0, vRot, numVals);              % Discretization for GR
            v(1:numVals) = vGround(1:numVals);
            aoa(1:numVals) = aoaSet;                                        % Good
            h(1:numVals)   = zeros(numVals, 1);                             % Good
            for i = 1:numVals
                [thrust(i), pUse(i)] = ThrustCalc(v(i), throttle);
                q(i)  = 0.5*rho(i)*v(i)^2;                            % Good
                CL(i) = CalcCL(aoa(i));                                     % Need function
                CD(i) = CD0 + k*CL(i)^2;                                    % Good
                LD(i) = CL(i)/CD(i);                                        % Good
                L(i)  = CL*Sref*q(i);                                       % Good
                D(i)  = (L(i)/LD(i)) + ((mass*g-L(i))*grFriction);           % Good
                a = (thrust(i) - D(i))/m;                                   % Good
                if i == 1
                    dt(1) = 0;                                              % dt starts at 0
                    t(1)  = 0;                                              % Time starts at 0
                    d(1)  = 0;                                              % Distance starts at 0
                else
                    dt(i)         = (v(i)-v(i-1))/(0.5*(a(i-1)+a(i)));      % Good
                    t(i)          = t(i-1) + dt(i);                         % Good
                    d(i)          = 0.5*(vGround(i-1)+vGround(i))*dt(i);    % Good
                    Energy(i) = Energy(i-1) + 0.5*(pUse(i-1)+pUse(i))*dt;   % Good
                end
            end


        % Rotation
            vGround(numVals+1 : numVals*2) = linspace(vRot, vLOF, numVals); % Discretized from vRot to vLOF
            v(numVals+1 : numVals*2) = vGround(numVals+1 : numVals*2);
            aoa(numVals+1 : numVals*2) = linspace(0, aoaLOF, numVals);      % Assume constant rotation rate
            h(numVals+1 : numVals*2) = zeros(numVals, 1);                   % Good
            for i = numVals+1 : numVals*2
                [thrust(i), pUse(i)] = ThrustCalc(v(i), throttle(i));
                q(i)  = 0.5*rho(i)*v(i)^2;                                  % Good
                CL(i) = CalcCL(aoa(i));                                     % Need function. Changing CL.
                CD(i) = CD0 + k*CL(i)^2;                                    % Good
                LD(i) = CL(i)/CD(i);                                        % Good
                L(i)  = CL*Sref*q(i);                                       % Good
                D(i)  = (L(i)/LD(i)) + ((mass*g-L(i))*grFriction);           % Good
                a = (thrust(i) - D(i))/m;                                   % Good
                dt(i)         = (v(i)-v(i-1))/(0.5*(a(i-1)+a(i)));      % Good
                t(i)          = t(i-1) + dt(i);                         % Good
                d(i)          = 0.5*(vGround(i-1)+vGround(i))*dt(i);                % Good
                Energy(i) = Energy(i-1) + 0.5*(pUse(i-1)+pUse(i))*dt;   % Good
            end


        % Transition
        % Notes:
        % - Separate vGround and hDot. Discretized over v. Use fpa to solve
            v(2*numVals+1 : numVals*3) = linspace(vLOF, vClimb, numVals);   % Discretized from vRot to vLOF
            vGroundClimb = sqrt(vClimb^2 - hDotClimb^2);                    % Ground speed at end of discretization
            fpaClimb     = atan2(hDotClimb, vGroundClimb);                  % fpa at end of discretization
            fpa          = linspace(0, fpaClimb, numVals);                  % Assume constant rotation rate
            vGround      = v(2*numVals+1 : numVals*3) .* cos(fpa);

        for i = 1:numVals
            h(i) = integrateHDot; % Do this
            % t Energy pUse d a h hDot thrust q CL CD L D LD
        end

        % New structure/table
            Snew.Time   = t;
            Snew.E      = Energy;
            Snew.Power  = pUse;
            Snew.Distance = d;
            Snew.V_Ground = vGround;
            Snew.V = v;
            Snew.Acceleration = a;
            Snew.Altitude = h;
            Snew.hDot   = hDot;
            Snew.Mass   = mass*ones(numVals,1);
            Snew.Thrust = thrust;
            Snew.q      = q;
            Snew.CL     = CL;
            Snew.CD     = CD;
            Snew.Lift   = L;
            Snew.Drag   = D;
            Snew.LD     = LD;
            Tnew = struct2table(Snew); % Make it a structure
            MissionTable = [tab; Tnew]; % Concatenate tables
        end
    end
end

