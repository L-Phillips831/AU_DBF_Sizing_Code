classdef Takeoff < mission.Mission_Segment
    %TAKEOFF Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        vehicle Vehicle
        numVals (1,1) double
        T_set (1,1) double
        T_set_climb (1,1) double
        vWind_NED double
    end
    
    methods
        function obj = Takeoff(T_set, T_set_climb, numVals, vehicle, vWind_NED)
            obj.T_set = T_set;
            obj.T_set_climb = T_set_climb;
            obj.vehicle = vehicle;
            obj.numVals = numVals;
            obj.vWind_NED = vWind_NED;

           
        end

        function tbl = run(obj, tab)
        % TAKEOFF Construct an instance of this class

        % General variables
            Sref     = obj.vehicle.S_ref;
            k1       = obj.vehicle.K1;
            k2       = obj.vehicle.K2;
            Cd_0     = obj.vehicle.CD_0;
            rho      = obj.vehicle.rho;
            Cl_max   = obj.vehicle.Cl_max;

        % Set/Initial values
            vWind_NED_Start = obj.vWind_NED;
            T_set_           = obj.T_set;
            grFriction      = 0.05;
            k_TO            = 1.3;
            g               = 9.81; 
            mass            = obj.vehicle.mass;
            numVals_        = obj.numVals;
            numParts        = 2; % for a taildragger with no rotation
            weight          = mass * g;
            vLOF            = sqrt(2*weight / (Cl_max * Sref * rho))*k_TO;
            aoaSet          = obj.vehicle.get_req_alpha( Cl_max);   

        % Initialize table variables
            vAir_NED   = zeros(numParts*numVals_, 3);
            v_NED      = zeros(numParts*numVals_, 3);
            vWind_NED_ = repmat(vWind_NED_Start, numParts*numVals_, 1);
            eulers    = zeros(numParts*numVals_, 3);
            pos_NED   = zeros(numParts*numVals_, 3);
            throttle  = T_set_ * ones(numParts*numVals_, 1);
            thrust    = zeros(numParts*numVals_, 1);
            q         = zeros(numParts*numVals_, 1);
            CL        = zeros(numParts*numVals_, 1);
            CD        = zeros(numParts*numVals_, 1);
            alpha     = zeros(numParts*numVals_, 1);
            gamma     = zeros(numParts*numVals_, 1);
            E         = zeros(numParts*numVals_, 1);
            power     = zeros(numParts*numVals_, 1);
            t         = zeros(numParts*numVals_, 1);

        % Ground Run
            v_Air_Static            = [0, 0, 0] - vWind_NED_Start;
            vAir_NED_x              = linspace(v_Air_Static(1), vLOF, numVals_);
            vAir_NED(1:numVals_, 1) = vAir_NED_x;
            v_NED(1:numVals_, :)    = vAir_NED(1:numVals_, :) + vWind_NED_Start;
            alpha(1:numVals_)       = aoaSet;
            eulers(1:numVals_,:)    = repmat([0, aoaSet, 0], numVals_, 1);
            q(1:numVals_)           = 0.5 * rho * vAir_NED_x.^2;
            CL(1:numVals_)          = Cl_max;
            CD(1:numVals_)          = Cd_0 + k1*CL(1:numVals_) + k2*CL(1:numVals_).^2;

            dv = vAir_NED_x(2) - vAir_NED_x(1);

            for i = 1:numVals_
                if vAir_NED(i, 1) < 0
                    vAir_NED(i, 1) = 0;
                end                
                BI        = Vehicle.get_DCM_BI(eulers(i, 1), eulers(i, 2), eulers(i, 3));
                vAero     = sqrt(sum(vAir_NED(i, :).^2));
                q(i)      = 0.5 * rho * vAero^2;
                vAir_Body = BI*vAir_NED(i,:)';
                RPM = obj.vehicle.prop.calcRPM(obj.T_set);
                [thrust(i), power(i)] = obj.vehicle.prop.get_Thrust_Power(RPM, vAir_Body(1));
                lift  = CL(i)*Sref*q(i);
                drag  = CD(i)*Sref*q(i) + grFriction * (weight - lift); % Make sure drag is not >=
                a_NED = [thrust(i) - drag, 0, 0]/mass;
                dt    = dv / a_NED(1);
                if i >1
                    t(i) = t(i-1) + dt;
                    pos_NED(i, 1) = trapz(t(1:i), v_NED(1:i, 1));
                    E(i) = trapz(t(1:i), power(1:i));
                end
            end

%{
        % Rotation
            vGround(numVals_+1 : numVals_*2) = linspace(vRot, vLOF, numVals_); % Discretized from vRot to vLOF
            v(numVals_+1 : numVals_*2) = vGround(numVals_+1 : numVals_*2);      % Airspeed same as ground speed for rotation phase
            aoa(numVals_+1 : numVals_*2) = linspace(aoaSet, aoaLOF, numVals_);      % Assume constant rotation rate
            h(numVals_+1 : numVals_*2) = zeros(numVals_, 1);                  
            for i = numVals_+1 : numVals_*2
            [power(i), thrust(i)] = obj.Propulsion.get_Thrust(obj.T_set, v(i));
                q(i)  = 0.5*rho(i)*v(i)^2;                                  
                CL(i) = CalcCL(aoa(i));                                     % Have not made this. would need Cl_0 and Cl_alpha constants
                CD(i) = Cd_0 + k*CL(i)^2;                                    
                LD(i) = CL(i)/CD(i);                                        
                L(i)  = CL*Sref*q(i);                                       
                D(i)  = (L(i)/LD(i)) + ((mass*g-L(i))*grFriction);          % Drag is aerodynamic drag plus ground friction force
                a(i) = (thrust(i) - D(i))/mass;
                dt(i)         = (v(i)-v(i-1))/(0.5*(a(i-1)+a(i))); 
                t(i)          = t(i-1) + dt(i);                         
                d(i)          = 0.5*(vGround(i-1)+vGround(i))*dt(i);             
                Energy(i) = Energy(i-1) + 0.5*(pUse(i-1)+pUse(i))*dt;
            end
%}

        % Climb Transition
            % Find best vClimb and gamma_Air
            v_opts = linspace(0, 25, numVals_);
            hDots = zeros(numVals_, 1);
            CLs = zeros(numVals_, 1);
            for i = 1:numVals_
                RPM = obj.vehicle.prop.calcRPM(obj.T_set_climb);
                [t_Pos,~]    = obj.vehicle.prop.get_Thrust_Power(RPM, v_opts(i));
                CLs(i)       = weight / (Sref * 0.5 * rho * v_opts(i)^2);
                Cd       = Cd_0 + k1*CLs(i) + k2*CLs(i)^2;
                d        = Cd * Sref * 0.5 * rho * v_opts(i)^2;
                hDots(i) = v_opts(i) * (t_Pos-d) / weight;
            end
            [hDot_climb, idx] = max(hDots);
            vClimb            = v_opts(idx);
            gamma_Air_Climb   = asind(hDot_climb / vClimb);
            CL_climb          = CLs(idx);
            alpha_climb       = obj.vehicle.get_req_alpha( CL_climb);
            
            % Discretizations:
            vAir_total = linspace(vLOF, vClimb, numVals_);
            gamma_Air  = linspace(0, gamma_Air_Climb, numVals_);
            alpha((numVals_ + 1):(2*numVals_)) = linspace(alpha(numVals_), alpha_climb, numVals_);
            CL((numVals_ + 1):(2*numVals_)) = linspace(CL(numVals_), CL_climb, numVals_);
            n_Trans         = 1.2;
            R = (vLOF + vClimb)^2 / (4*g*(n_Trans-1));

            for i = (numVals_ + 1):(2*numVals_)
                if (i-numVals_) > 1
                    t(i) = t(i-1) + dt;
                else
                    t(i) = t(i-1);
                end
                gammaDot = 180/pi*vAir_total(i-numVals_)/R;
                dt = (gamma_Air(2) - gamma_Air(1))/gammaDot;
                q(i)     = 0.5 * rho * vAir_total(i-numVals_)^2;
                CD(i)    = Cd_0 + k1*CL(i)^2;
                RPM = obj.vehicle.prop.calcRPM(obj.T_set);
                [thrust(i), power(i)] = obj.vehicle.prop.get_Thrust_Power(RPM, vAir_total(i-numVals_));
                pSpec = vAir_total(i-numVals_)*(thrust(i)-CD(i)*Sref*q(i))/weight; % Added this guy
                dvTotal = vAir_total(2) - vAir_total(1);
                pSpec_Z = pSpec - vAir_total(i-numVals_)/g*dvTotal/dt;
                vAir_NED(i, 3) = - pSpec_Z;
                vAir_NED(i, 1) = vAir_total(i-numVals_)*cosd(gamma_Air(i-numVals_));
                v_NED(i, :)    = vWind_NED_(i, :) + vAir_NED(i, :);
                gamma(i)       = atand(- v_NED(i, 3) / v_NED(i, 1));
                eulers(i, :)   = [0, gamma(i) + alpha(i), 0];
                if (i-numVals_) > 1
                    pos_NED(i, 1)  = trapz(t(1:i), v_NED(1:i, 1));
                    pos_NED(i, 2)  = trapz(t(1:i), v_NED(1:i, 2));
                    pos_NED(i, 3)  = trapz(t(1:i), v_NED(1:i, 3));
                    E(i)           = trapz(t(1:i), power(1:i));
                else
                    pos_NED(i, 1)  = pos_NED(i-1, 1);
                    pos_NED(i, 2)  = pos_NED(i-1, 2);
                    pos_NED(i, 3)  = pos_NED(i-1, 3);
                    E(i)           = E(i-1);                  
                end
            end

        % New structure/table. All vectors in NED
            Snew.Airspeed_NED    = vAir_NED;
            Snew.Groundspeed_NED = v_NED;
            Snew.Windspeed_NED   = vWind_NED_;
            Snew.Eulers          = eulers;
            Snew.Position_NED    = pos_NED;
            Snew.Mass            = mass*ones(numParts*numVals_,1);
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
            tbl = [tab; struct2table(Snew)]; % Make it a structure
        end
    end
end

