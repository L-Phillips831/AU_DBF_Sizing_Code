classdef Turn < mission.Mission_Segment
    %TURN Summary of this class goes here
    %   Detailed explanation goes here

    % To do:
    % - Bring in all necessary variables
    % - Find a way to track alpha well. Otherwise, set to 0
    % - rework sustained turn to take thrust from throttle setting and
    % get the turn properties achieved form that.
    % - down is positive
    % - Take wind out of the equation
    
    properties       
        vehicle Vehicle
        dPsi (1,1) double
        numVals (1,1) double
        option char
        T_set (1,1) double

    end
    
    methods

        function obj = Turn(T_set, dPsi, numVals, vehicle, option)

            obj.T_set = T_set;
            obj.vehicle = vehicle;
            obj.dPsi = dPsi;
            obj.numVals = numVals;

             switch option
                case {'instantaneous', 'Instantaneous'}
                    obj.option = option;

                case {'sustained', 'Sustained'}
                    obj.option = option;

                otherwise  
                    error("Option must be (I/i)nstantaneous or (S/s)ustained");

            end

        end

        function tbl = run(obj, tab)
            
            switch obj.option
                case {'instantaneous', 'Instantaneous'}
                    tbl = obj.InstantaneousTurn(tab);

                case {'sustained', 'Sustained'}
                    tbl = obj.SustainedTurn(tab);

                otherwise  
                    error("Option must be (I/i)nstantaneous or (S/s)ustained");

            end

        end
        
        
        function tbl = InstantaneousTurn(obj, tab)
        % Instantaneous Turn Mission Segment

        % Description: Takes heading change and a flight history table as
        % inputs. Finds the minimum lift bound between structural and
        % aerodynamic maximum lift available. Gets centripetal acceleration
        % as Lift/mass. Gets thrust and power from throttle setting and
        % velocity. Gets forward acceleration from (T-D)/mass. Turn radius
        % not fixed because speed and lift vary.

        % General variables
            Sref     = obj.vehicle.S_ref;                                              
            k2        = obj.vehicle.K2;
            k1       = obj.vehicle.K1;
            Cd_0     = obj.vehicle.CD_0;                                                
            rho      = obj.vehicle.rho;  
            g        = 9.81;     % Acceleration due to gravity [m/s^2] 
            numVals_ = obj.numVals;
            T_set_   = obj.T_set;
            Cl_max   = obj.vehicle.Cl_max;
            dPsi_    = obj.dPsi;

        % Instantaneous variables
            lfStruc = obj.vehicle.lfStruc;                                  % Need call, maximum structural load factor

        % Pull start of segment conditions
            vAir_NED_Start  = tab.Airspeed_NED(end, :);
            v_NED_Start     = tab.Groundspeed_NED(end, :);
            vWind_NED_Start = tab.Windspeed_NED(end, :);
            pos_NED_Start   = tab.Position_NED(end, :);
            tStart          = tab.Time(end);
            psi_Start       = tab.Eulers(end, 3);
            E_Start         = tab.Energy(end);

        % Initialize table variables
            t         = tStart*ones(numVals_, 1);
            vAir_NED  = repmat(vAir_NED_Start, numVals_, 1); 
            v_NED     = repmat(v_NED_Start, numVals_, 1); 
            vWind_NED = repmat(vWind_NED_Start, numVals_, 1); 
            pos_NED   = repmat(pos_NED_Start, numVals_, 1);
            throttle  = obj.T_set*ones(numVals_, 1);
            thrust    = zeros(numVals_, 1);
            q         = zeros(numVals_, 1);
            CL        = zeros(numVals_, 1);
            CD        = zeros(numVals_, 1);
            mass      = (tab.Mass(end));                   
            power     = zeros(numVals_,1);
            E         = E_Start*ones(numVals_, 1);
            alpha     = zeros(numVals_,1);
            eulers    = zeros(numVals_,3);
            gamma     = zeros(numVals_,1);                  


        % Discretization on heading
            psi = psi_Start + linspace(0, dPsi_, numVals_);

        % Heading wrap-around -180 to 180
            for i = 1:numVals_
                if psi(i) < -180
                    psi(i) = psi(i) + 360;
                elseif psi(i) > 180
                    psi(i) = psi(i) - 360;
                end
            end

        % Euler angles set
            eulers(:, 3) = psi;
            if dPsi_ > 0
                eulers(:, 1) = 90; % Bank right to go right
            elseif dPsi_ < 0
                eulers(:, 1) = -90; % Bank left to go left
            end

            weight = mass * g;

        % Instantaneous
            for i = 1:numVals_
                alpha(i)     = 0; % Small angle approx.
                eulers(i, 2) = alpha(i);
                if i > 1
                    v_NED(i, :) = v_NED(i-1, :) + a_NED' * dt;
                    gamma(i)    = atand(v_NED(i, 3) / sqrt(v_NED(i, 1)^2 + v_NED(i, 2)^2));
                end
                BI = Vehicle.get_DCM_BI(eulers(i, 1), eulers(i, 2), eulers(i, 3));
                IB = BI';
                vAir_NED(i, :) = v_NED(i, :);
                vAero          = sqrt(vAir_NED(i, 1)^2 + vAir_NED(i, 2)^2);
                q(i)           = 0.5*rho*vAero^2;
                LmaxAero       = Cl_max*q(i)*Sref;
                LmaxStructure  = lfStruc*mass*g;
                L              = min([LmaxAero, LmaxStructure]);                         % Constrained by aero or structure
                CL(i)          = L/(Sref*q(i));
                CD(i)          = Cd_0 + k1*CL(i) + k2*CL(i)^2;
                D              = CD(i) * q(i) * Sref;
                vAir_Body      = BI * vAir_NED(i, :)';
                [thrust(i), power(i)] = obj.vehicle.Propulsion.get_Thrust_Power(T_set_, vAir_Body(1));
                [F_x_Body, F_z_Body]  = Vehicle.reconcile_L_D(L, D, alpha(i));
                F_x_Body       = thrust(i) + F_x_Body;
                F_y_Body       = weight * sin(eulers(i, 1));
                a_Body         = [F_x_Body; F_y_Body; F_z_Body]/mass;
                a_NED          = IB * a_Body;
                r              = vAero^2 / a_Body(3);
                omega          = 180/pi*(vAero/r);
                dt = abs((psi(2)-psi(1))/omega);
                if i>1
                    t(i) = t(i-1) + dt;
                    E(i)         = E_Start + trapz(t(1:i) , power(1:i));
                    pos_NED(i,1) = pos_NED_Start(1) + trapz(t(1:i) , v_NED(1:i, 1));
                    pos_NED(i,2) = pos_NED_Start(2) + trapz(t(1:i) , v_NED(1:i, 2));
                end
                pos_NED(i,3) = pos_NED_Start(3) - 0.5 * g * (t(i) - t(1))^2;
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
            Tnew = struct2table(Snew); % Make it a structure
            tbl = [tab; Tnew]; % Concatenate tables and return
        end



        function tbl = SustainedTurn(obj, tab)
            % Sustained Turn Mission Segment

            % Description: Takes heading change and the aircraft object.
            % Aircraft object contains a table of flight time history.
            % Gets lift from the sustained load factor. Centripetal
            % acceleration is found from g and the sustained lf. Thrust
            % equals drag and propPower = Thrust*v. Gets pUse from the
            % propulsion model with input propPower.

            % Note:
            % - Alpha assumed 0 to match thrust/drag and lift/weight

        % General variables
            Sref     = obj.vehicle.S_ref;  
            % K1        = obj.vehicle.K1;  
            K2        = obj.vehicle.K2;                                                  
            Cd_0     = obj.vehicle.CD_0;                                                
            rho      = obj.vehicle.rho;                                       
            g        = 9.81; 
            lfStruc  = obj.vehicle.lf_struc;
            k_Safe_  = 1.5;
            numVals_ = obj.numVals;
            Cl_max   = obj.vehicle.Cl_max;
            dPsi_    = obj.dPsi;

        % Pull start of segment conditions
            vAir_NED_Start  = tab.Airspeed_NED(end, :);
            v_NED_Start     = tab.Groundspeed_NED(end, :);
            vWind_NED_Start = tab.Windspeed_NED(end, :);
            pos_NED_Start   = tab.Position_NED(end, :);
            tStart          = tab.Time(end);
            psi_Start       = tab.Eulers(end, 3);
            E_Start         = tab.Energy(end);            

            % Initialize table variables
            t         = tStart*ones(numVals_, 1);
            vAir_NED  = repmat(vAir_NED_Start, numVals_, 1);
            v_NED     = repmat(v_NED_Start, numVals_, 1);
            vWind_NED = repmat(vWind_NED_Start, numVals_, 1);
            pos_NED   = repmat(pos_NED_Start, numVals_, 1);
            throttle  = obj.T_set*ones(numVals_, 1);
            thrust    = zeros(numVals_, 1);
            q         = zeros(numVals_, 1);
            CL        = zeros(numVals_, 1);
            CD        = zeros(numVals_, 1);
            mass      = (tab.Mass(end));                   
            power     = zeros(numVals_,1);
            E         = E_Start*ones(numVals_, 1);
            alpha     = zeros(numVals_,1);
            eulers    = zeros(numVals_,3);
            gamma     = zeros(numVals_,1);  

            v_NED(:, 3) = 0;

            % Discretization on heading
            psi = psi_Start + linspace(0, dPsi_, numVals_);

            % Heading wrap-around -180 to 180
            for i = 1:numVals_
                if psi(i) < -180
                    psi(i) = psi(i) + 360;
                elseif psi(i) > 180
                    psi(i) = psi(i) - 360;
                end
            end

            % Euler angles set
            eulers(:, 3) = psi;

            weight = mass * g;

            % Calculate Sustained turn constant values
            v_Calc = sqrt(sum(vAir_NED_Start.^2));
            RPM = obj.vehicle.prop.calcRPM(obj.T_set);
            [thrust_Val, power_Val] = obj.vehicle.prop.get_Thrust_Power(RPM, v_Calc);
            q_Val    = 0.5 * rho * v_Calc^2;
            CD_Val   = thrust_Val / (Sref * q_Val);
            CL_Val   = sqrt((CD_Val - Cd_0)/K2);
            CL_Use   = min(CL_Val, Cl_max);
            lift_Val = CL_Use * q_Val * Sref;
            lf_Aero  = lift_Val / (weight * k_Safe_^2);
            lf_Use   = min(lfStruc, lf_Aero);

            for i = 1:numVals_
                q(i)         = q_Val;
                CL(i)        = CL_Use;
                CD(i)        = CD_Val;
                thrust(i)    = thrust_Val;
                power(i)     = power_Val;
                alpha(i)     = 0;
                eulers(i, 2) = alpha(i); % Assumed 0, small angle approx.
                if dPsi_ > 0
                    eulers(i, 1) = acosd(1/lf_Use); % Bank right to go right
                elseif dPsi_ < 0
                    eulers(i, 1) = -acosd(1/lf_Use); % Bank left to go left
                end
                v_NED(i,:) = v_Calc * [cosd(eulers(i,3)), sind(eulers(i, 3)), 0];
                vAir_NED(i, :) = v_NED(i, :);
                aCentr = g*sqrt(lf_Use^2 - 1);
                omega  = 180/pi*aCentr/v_Calc; 
                if i < numVals_
                    dt = abs((eulers(5, 3)-eulers(4, 3))/omega);
                end
                if i>1
                    t(i) = t(i-1)+ dt;
                    E(i)         = E_Start + (t(i) - t(1)) * power(i);
                    pos_NED(i,1) = pos_NED_Start(1) + trapz(t(1:i) , v_NED(1:i, 1));
                    pos_NED(i,2) = pos_NED_Start(2) + trapz(t(1:i) , v_NED(1:i, 2));
                end
                pos_NED(i,3) = pos_NED_Start(3);
            end
            
            pos_NED(end, 2) = 0;

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
            Tnew = struct2table(Snew); % Make it a structure
            tbl = [tab; Tnew]; % Concatenate tables and return
        end
    end
end

