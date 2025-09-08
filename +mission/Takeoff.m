classdef Takeoff < mission.Mission_Segment
    %TAKEOFF Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        vehicle (1,1) Vehicle
        numVals (1,1) double
    end
    
    methods
        function obj = Takeoff(T_set, numVals, vehicle)
            obj.T_set = T_set;
            obj.vehicle = vehicle;
            obj.numVals_ = numVals;
           
        end

        function tbl = run(obj)
        % TAKEOFF Construct an instance of this class
        %{
        - Will have ground run and transition
        %}


        % To Do:
        % - Make sure that thrust, lift, drag, and weight act in their
        % respective directions for ground run + rotation
        % - Put in the correct thrust/propulsion functions
        % - down is positive
        % KEEP ADDING wind/air speed


        % General variables
            Sref = obj.Sref;
            k    = obj.k;
            CD0  = obj.CD0;
            rho  = obj.rho;
            g = obj.g;

        % Set/Initial values
            vWind_NED_Start = obj.vWind_NED;
            vRot = obj.vRot;
            vLOF = obj.vLOF;
            aoaLOF = obj.aoaLOF;
            vClimb = obj.vClimb;
            T_set = obj.T_set;
            aoaSet = obj.aoaSet; % aoa at forward ground run
            grFriction = obj.grFriction;
            mass = obj.mass;
            numVals_ = obj.numVals;
            numParts = 2; % for a taildragger with no rotation
            tTrans = 2;     % time (s) it takes to transition from fpa 0 to fpaClimb and vLOF to vClimb. May need some logic to be iteratively increased if throttle is above 1

            % Initialize table variables
            vAir_NED  = zeros(numParts*numVals_, 3);
            v_NED     = zeros(numParts*numVals_, 3);
            vWind_NED = repmat(vWind_NED_Start, numParts*numVals_, 1);
            eulers    = zeros(numParts*numVals_, 3);
            pos_NED   = zeros(numParts*numVals_, 3);
            throttle  = T_set * ones(numParts*numVals_, 1);
            thrust    = zeros(numParts*numVals_, 1);
            q         = zeros(numParts*numVals_, 1);
            CL        = zeros(numParts*numVals_, 1);
            CD        = zeros(numParts*numVals_, 1);
            alpha     = zeros(numParts*numVals_, 1);
            gamma     = zeros(numParts*numVals_, 1);
            E         = zeros(numParts*numVals_, 1);
            power     = zeros(numParts*numVals_, 1);
            t         = zeros(numParts*numVals_, 1);

        % Ground Run (NEW)
            v_Air_Static = 0 - vWind_NED_Start;
            vAir_NED_x = linspace(v_Air_Static, vRot, numVals_);
            vAir_NED(1:numVals_, 1) = vAir_NED_x;
            v_NED(1:numVals_, :) = vAir_NED + vWind_NED;
            aoa(1:numVals_) = aoaSet;
            q(1:numVals_) = 0.5 * rho * vAir_NED_x.^2;
            CL(1:numVals_) = CL0 + CL_alpha*aoaSet;
            CD(1:numVals_) = CD0 + k*CL(1:numVals_).^2;

            dv = vAir_NED_x(2) - vAir_NED_x(1);

            for i = 1:numVals_
            %{
FOR LOOP:
power and thrust from propulsion
drag from aerodynamics and ground friction
acceleration NED from thrust and drag
time updated with dv / acceleration
position updated with velocity
Energy with trapz

            %}
            end



        % Rotation
            vGround(numVals_+1 : numVals_*2) = linspace(vRot, vLOF, numVals_); % Discretized from vRot to vLOF
            v(numVals_+1 : numVals_*2) = vGround(numVals_+1 : numVals_*2);      % Airspeed same as ground speed for rotation phase
            aoa(numVals_+1 : numVals_*2) = linspace(aoaSet, aoaLOF, numVals_);      % Assume constant rotation rate
            h(numVals_+1 : numVals_*2) = zeros(numVals_, 1);                  
            for i = numVals_+1 : numVals_*2
            [power(i), thrust(i)] = obj.Propulsion.get_Thrust(obj.T_set, v(i));
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
            v(2*numVals_+1 : numVals_*3) = linspace(vLOF, vClimb, numVals_);   % Discretized from vLOF to vClimb
            vGroundClimb = sqrt(vClimb^2 - hDotClimb^2);                    % Ground speed at end of discretization
            fpaClimb     = atan2(hDotClimb, vGroundClimb);                  % fpa at end of discretization
            fpaTrans          = linspace(0, fpaClimb, numVals_);                  % Assume constant transition rate
            dtTrans = tTrans / (numVals_ - 1);
            dvTrans = v(2*numVals_ + 2) - v(2*numVals_ + 1);
            
            for i = 2*numVals_+1 : numVals_*3
            vGround(i) = v(i) .* cos(fpaTrans(i));
            hDot(i) = v(i).*sin(fpaTrans(i));
            a(i) = ones(numVals_, 1) * dvTrans/dtTrans;
            a_X = a(i) * cos(fpaTrans(i));
            a_Y = a(i) * sin(fpaTrans(i));
            q(i) = 0.5*rho*v(i)^2;
            CL(i) = CL(2*numVals_); % Assume same CL as liftoff
            CD(i) = CD(2*numVals_); % Assume same CD as liftoff
            LD(i) = CL(i) / CD(i);
            L(i) = q(i) * CL(i) * Sref;
            D(i) = q(i) * CD(i) * Sref;
            T_cos_aoa = mass*a_X + L*sin(fpaTrans(i)) - D*cos(fpaTrans(i));
            T_sin_aoa = mass*(a_Y +g) + D*sin(fpaTrans(i)) - L*cos(fpaTrans(i));
            aoa(i) = atan(T_sin_aoa/T_cos_aoa);
            thrust(i) = T_cos_aoa(i) / cos(aoa(i));
            [throttle(i), pUse(i)] = GetThrottle(thrust(i), v(i)); % Not the proper function call.
            % Check that throttle is not above 1. If so, lengthen transition.
            if i > 2*numVals_ + 1
                t(i) = t(i-1) + dtTrans;
                Energy(i) = Energy(i-1) + pUse(i)*dtTrans;
            else
                t(i) = t(i-1);
                Energy(i) = Energy(i-1);
            end
            h(i) = cumtrapz(t, hDot);
            d(i) = d(2*numVals_) + cumtrapz(t(2*numVals_+1:i), vGround(2*numVals_+1:i));
            
            end

        % New structure/table. All vectors in NED
            Snew.Airspeed_NED    = vAir_NED;
            Snew.Groundspeed_NED = v_NED;
            Snew.Windspeed_NED   = vWind_NED;
            Snew.Eulers          = eulers;
            Snew.Position_NED    = pos_NED;
            Snew.Mass            = mass*ones(numVals_,1);
            Snew.Throttle        = throttle;
            Snew.Thrust_Body     = thrust;
            Snew.q               = q;
            Snew.CL              = CL;
            Snew.CD              = CD;
            Snew.Alpha           = alpha;
            Snew.Gamma           = gamma;
            Snew.Energy          = E;
            Snew.Power           = power;
            Snew.Time            = t;
            tbl = struct2table(Snew); % Make it a structure
        end
    end
end

