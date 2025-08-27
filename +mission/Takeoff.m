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
            nVals   = obj.nVals;

            numParts  = 3;                                                  % Ground run, rotation, transition
            tTrans = 2;                                                % time (s) it takes to transition from fpa 0 to fpaClimb and vLOF to vClimb

            % Initialize table variables
            t       = zeros(numParts*nVals, 1);
            d       = zeros(numParts*nVals, 1);
            vGround = zeros(numParts*nVals, 1);
            v       = zeros(numParts*nVals, 1);
            a       = zeros(numParts*nVals, 1);
            aoa     = zeros((numParts-1)*nVals, 1);
            pUse    = zeros(numParts*nVals, 1);
            thrust  = zeros(numParts*nVals, 1);
            q       = zeros(numParts*nVals, 1);
            CL      = zeros(numParts*nVals, 1);
            CD      = zeros(numParts*nVals, 1);
            L       = zeros(numParts*nVals, 1);
            D       = zeros(numParts*nVals, 1);
            LD      = zeros(numParts*nVals, 1);
            hDot    = zeros(numParts*nVals, 1);
            h       = zeros(numParts*nVals, 1); 
            Energy  = zeros(numParts*nVals, 1);

            % Other initializations
            dt = zeros(numParts*nVals,1);

            % Ground run
            vGround(1:nVals) = linspace(0, vRot, nVals);              % Discretization for GR
            v(1:nVals)       = vGround(1:nVals);
            aoa(1:nVals)     = aoaSet;                                        
            h(1:nVals)       = zeros(nVals, 1);                             
            for i = 1:nVals
                [thrust(i), pUse(i)] = ThrustCalc(v(i), throttle); % ThrustCalc is a function needed from Cole's propulsion
                q(i)  = 0.5*rho(i)*v(i)^2;                            
                CL(i) = CalcCL(aoa(i));                                     % Have not made this. would need CL0 and CL_Alpha constants
                CD(i) = CD0 + k*CL(i)^2;                                    
                LD(i) = CL(i)/CD(i);                                        
                L(i)  = CL*Sref*q(i);                                       
                D(i)  = (L(i)/LD(i)) + ((mass*g-L(i))*grFriction);          % Drag is aerodynamic drag plus ground friction force
                a(i) = (thrust(i) - D(i))/mass;                                   
                if i > 1
                    dt(i)         = (v(i)-v(i-1))/(0.5*(a(i-1)+a(i)));      
                    t(i)          = t(i-1) + dt(i);                         
                    d(i)          = 0.5*(vGround(i-1)+vGround(i))*dt(i);    
                    Energy(i) = Energy(i-1) + 0.5*(pUse(i-1)+pUse(i))*dt;
                end
            end


        % Rotation
            vGround(nVals+1 : nVals*2) = linspace(vRot, vLOF, nVals); % Discretized from vRot to vLOF
            v(nVals+1 : nVals*2) = vGround(nVals+1 : nVals*2);      % Airspeed same as ground speed for rotation phase
            aoa(nVals+1 : nVals*2) = linspace(aoaSet, aoaLOF, nVals);      % Assume constant rotation rate
            h(nVals+1 : nVals*2) = zeros(nVals, 1);                  
            for i = nVals+1 : nVals*2
                [thrust(i), pUse(i)] = ThrustCalc(v(i), throttle); % ThrustCalc is a function needed from Cole's propulsion
                q(i)  = 0.5*rho(i)*v(i)^2;                                  
                CL(i) = CalcCL(aoa(i));                                     % Have not made this. would need CL0 and CL_Alpha constants
                CD(i) = CD0 + k*CL(i)^2;                                    
                LD(i) = CL(i)/CD(i);                                        
                L(i)  = CL*Sref*q(i);                                       
                D(i)  = (L(i)/LD(i)) + ((mass*g-L(i))*grFriction);          % Drag is aerodynamic drag plus ground friction force
                a(i) = (thrust(i) - D(i))/mass;
                dt(i)         = (v(i)-v(i-1))/(0.5*(a(i-1)+a(i))); 
                t(i)          = t(i-1) + dt(i);                         
                d(i)          = 0.5*(vGround(i-1)+vGround(i))*dt(i);             
                Energy(i) = Energy(i-1) + 0.5*(pUse(i-1)+pUse(i))*dt;
            end


        % Transition
        % Notes:
        % - Separate vGround and hDot. Discretized over v. Use fpa to solve
            v(2*nVals+1 : nVals*3) = linspace(vLOF, vClimb, nVals);   % Discretized from vLOF to vClimb
            vGroundClimb = sqrt(vClimb^2 - hDotClimb^2);                    % Ground speed at end of discretization
            fpaClimb     = atan2(hDotClimb, vGroundClimb);                  % fpa at end of discretization
            fpaTrans          = linspace(0, fpaClimb, nVals);                  % Assume constant transition rate
            vGround(2*nVals+1 : nVals*3) = v(2*nVals+1 : nVals*3) .* cos(fpaTrans);
            dtTrans = tTrans / (nVals - 1);
            t(2*nVals+1 : nVals*3) = t(2*nVals) : dtTrans : (t(2*nVals) + dtTrans);
            dvTrans = v(2*nVals + 2) - v(2*nVals + 1);
            a(2*nVals+1 : nVals*3) = ones(nVals, 1) * dvTrans/dtTrans;
            a_X = a(2*nVals+1 : nVals*3) * cos(fpaTrans(2*nVals+1 : nVals*3));
            a_Y = a(2*nVals+1 : nVals*3) * sin(fpaTrans(2*nVals+1 : nVals*3));
            hDot(2*nVals+1 : nVals*3) = v(2*nVals+1 : nVals*3).*sin(fpaTrans);
            T_cos_aoa = mass*a_X + L*sin(fpaTrans()) - D*cos(fpaTrans());
            T_sin_aoa = mass*a_Y + mass*g + D*sin(fpaTrans()) - L*cos(fpaTrans());
            % aoa = atan(T*sin(aoa)/(T*cos(aoa))
            % T = T_cos_aoa / cos(aoa)
            % Get throttle from T and v
            % Check that throttle is not above 1. If so, lengthen
            % transition.

            aoa(2*nVals+1 : nVals*3) = 0; % Correct

        for i = 1:nVals
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
            Snew.Mass   = mass*ones(nVals,1);
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

