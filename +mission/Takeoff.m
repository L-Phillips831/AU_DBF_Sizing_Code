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
        - Does not account for sloped runway
        - Does not account for wind speed
        %}


        % To Do:
        % - Make sure that thrust, lift, drag, and weight act in their
        % respective directions for ground run + rotation
        % - Put in the correct thrust/propulsion functions


        % General variables
            tab  = obj.MissionTable;
            Sref = obj.Sref;
            k    = obj.k;
            CD0  = obj.CD0;
            rho  = obj.rho;
            g = obj.g;
            aoaSet = obj.aoaSet; % aoa at forward ground run
            grFriction = obj.grFriction;
            mass = tab.mass(end);
            nVals   = obj.nVals;

            numParts  = 3;                                                  % Ground run, rotation, transition
            tTrans = 2;     % time (s) it takes to transition from fpa 0 to fpaClimb and vLOF to vClimb. May need some logic to be iteratively increased if throttle is above 1

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
            vGround(1:nVals) = linspace(0, vRot, nVals);
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
            dtTrans = tTrans / (nVals - 1);
            dvTrans = v(2*nVals + 2) - v(2*nVals + 1);
            
            for i = 2*nVals+1 : nVals*3
            vGround(i) = v(i) .* cos(fpaTrans(i));
            hDot(i) = v(i).*sin(fpaTrans(i));
            a(i) = ones(nVals, 1) * dvTrans/dtTrans;
            a_X = a(i) * cos(fpaTrans(i));
            a_Y = a(i) * sin(fpaTrans(i));
            q(i) = 0.5*rho*v(i)^2;
            CL(i) = CL(2*nVals); % Assume same CL as liftoff
            CD(i) = CD(2*nVals); % Assume same CD as liftoff
            LD(i) = CL(i) / CD(i);
            L(i) = q(i) * CL(i) * Sref;
            D(i) = q(i) * CD(i) * Sref;
            T_cos_aoa = mass*a_X + L*sin(fpaTrans(i)) - D*cos(fpaTrans(i));
            T_sin_aoa = mass*(a_Y +g) + D*sin(fpaTrans(i)) - L*cos(fpaTrans(i));
            aoa(i) = atan(T_sin_aoa/T_cos_aoa);
            thrust(i) = T_cos_aoa(i) / cos(aoa(i));
            [throttle(i), pUse(i)] = GetThrottle(thrust(i), v(i)); % Not the proper function call.
            % Check that throttle is not above 1. If so, lengthen transition.
            if i > 2*nVals + 1
                t(i) = t(i-1) + dtTrans;
                Energy(i) = Energy(i-1) + pUse(i)*dtTrans;
            else
                t(i) = t(i-1);
                Energy(i) = Energy(i-1);
            end
            h(i) = cumtrapz(t, hDot);
            d(i) = d(2*nVals) + cumtrapz(t(2*nVals+1:i), vGround(2*nVals+1:i));
            
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

