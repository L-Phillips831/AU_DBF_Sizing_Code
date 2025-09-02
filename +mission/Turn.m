classdef Turn < mission.Mission_Segment
    %TURN Summary of this class goes here
    %   Detailed explanation goes here

    % To Do:
    % - Make sure euler angles are the correct rotations. Is aoa = theta?
    % - Integrate over position/velocity
    % - Resolve q
    % - All of sustained
    
    properties       
        vehicle (1,1) Vehicle
        dPsi (1,1) double
        numVals (1,1) double
        option char

    end
    
    methods

        function obj = Turn(dPsi, numVals, vehicle, option)

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
                    tbl = obj.InstantaneousTurn(obj.dPsi, tab);

                case {'sustained', 'Sustained'}
                    tbl = obj.SustainedTurn(obj.dPsi, tab);

                otherwise  
                    error("Option must be (I/i)nstantaneous or (S/s)ustained");

            end

        end
        
        
        function tbl = InstantaneousTurn(obj, dPsi, tab)
        % Instantaneous Turn Mission Segment

        % Description: Takes heading change and a flight history table as
        % inputs. Finds the minimum lift bound between structural and
        % aerodynamic maximum lift available. Gets centripetal acceleration
        % as Lift/mass. Gets thrust and power from throttle setting and
        % velocity. Gets forward acceleration from (T-D)/mass. Turn radius
        % not fixed because speed and lift vary.

        % General variables
            Sref = obj.vehicle.Sref;                                              
            k    = obj.vehicle.k;                                                  
            CD0  = obj.vehicle.CD0;                                                
            rho  = obj.vehicle.rho;                                        
            g    = 9.81;     % Acceleration due to gravity [m/s^2] 
            numVals_ = obj.numVals;

        % Instantaneous variables
            lfStruc = obj.vehicle.lfStruc;                                          % Need call, maximum structural load factor

        % Pull start of segment conditions
            vAir_NED_Start = tab.Airspeed_NED(end, :);
            v_NED_Start = tab.Groundspeed_NED(end, :);
            vWind_NED_Start = tab.Windspeed_NED(end, :);
            tStart = tab.Time(end);
            alpha_Start = tab.Alpha(end);
            psi_Start = tab.Eulers(end, 3);
            E_Start = tab.Energy(end);

        % Initialize table variables
            t = tStart*ones(numVals_, 1);
            vAir_NED = repmat(vAir_NED_Start, numVals_, 1); % nx3
            v_NED = repmat(v_NED_Start, numVals_, 1); % nx3
            vWind_NED = repmat(vWind_NED_Start, numVals_, 1); % nx3
            throttle = obj.T_set*ones(numVals_, 1);
            thrust=zeros(numVals_, 1);
            q=zeros(numVals_, 1);
            CL=zeros(numVals_, 1);
            CD=zeros(numVals_, 1);
            mass =(tab.mass(end));                   
            power = zeros(numVals_,1);
            E = E_Start*zeros(numVals_, 1);
            alpha = zeros(numVals_,1);
            eulers = zeros(numVals_,3);
            gamma = zeros(numVals_,1);                  


        % Discretization on heading
            psi = linspace(0, dPsi, obj.numVals);
            eulers(:, 3) = psi;

        % Instantaneous
            for i = 1:numVals_
                q(i) = 0.5*rho*v(i)^2; % X AND Y VELOCITIES COMBINED
                LmaxAero = obj.vehicle.CLmax*q(i)*Sref;
                LmaxStructure = lfStruc*mass*g;
                L = min([LmaxAero, LmaxStructure]);                         % Constrained by aero or structure
                a_C = L/mass;                                   
                CL(i) = L/(Sref*q(i));
                CD(i) = CD0 + k*CL(i)^2;
                D = CD(i) * q(i) * Sref;
                [thrust(i), power(i)] = PropulsionCalc(v(i), T_set);         % NEED FUNCTION
                a_T = (thrust(i) - D)/mass;                             % forward acceleration
                r = v(i)^2 / a_C;
                omega = v(i)/r;                                             % turn rate in rad/s
                if i>1
                    E(i) = E(i-1) + power*dt;
                    dt = (psi(i)-psi(i-1))/omega;
                    t(i) = t(i-1) + dt;
                    dPsi(i) = dPsi(i-1) + (psi(i) - psi(i-1))*r;                  % Small angle approx
                    hDot(i) = g*(t(i) - t(1));
                    h(i) = h(1) - 0.5*g*(t(i) - t(1)); % PUT IN POSITION
                end

                a_x_NED = a_T * cos(psi) - a_C * sin(psi); % NEED a_T / a_C
                a_y_NED = a_C * cos(psi) + a_T * sin(psi);
                a_NED = [a_x_NED, a_y_NED, 0];
                if i < numVals_
                    v_NED(i+1) = v_NED(i) + a_NED*dt; % Next time step velocity. Velocity after current time step acceleration
                end
                                
            end
            

        % New structure/table. All vectors in NED
            Snew.Airspeed_NED = vAir_NED;
            Snew.Groundspeed_NED = v_NED;
            Snew.Windspeed_NED = vWind_NED;
            Snew.Eulers = eulers;
            Snew.Position_NED = pos_NED;
            Snew.Mass = mass*ones(numVals_,1);
            Snew.Throttle = throttle;
            Snew.Thrust_Body = thrust;
            Snew.q = q;
            Snew.CL = CL;
            Snew.CD = CD;
            Snew.Alpha = alpha;
            Snew.Gamma = gamma;
            Snew.Energy = E;
            Snew.Power = power;
            Snew.Time = t;
            Tnew = struct2table(Snew); % Make it a structure
            tbl = [tab; Tnew]; % Concatenate tables and return
        end



        function MissionTable = SustainedTurn(obj, dPsi, tab)
            % Sustained Turn Mission Segment

            % Description: Takes heading change and the aircraft object.
            % Aircraft object contains a table of flight time history.
            % Gets lift from the sustained load factor. Centripetal
            % acceleration is found from g and the sustained lf. Thrust
            % equals drag and propPower = Thrust*v. Gets pUse from the
            % propulsion model with input propPower.

            % General variables
            Sref = obj.vehicle.Sref;                                                
            k    = obj.vehicle.k;                                                   
            CD0  = obj.vehicle.CD0;                                                
            rho  = obj.vehicle.rho;                                                 
            mass = tab.vehicle.mass(end);                                          
           
            g    = 9.81;                                                  
            lf   = obj.lfSust;                                              % Need call, sustained load factor


            % Initialize table variables
            t      = tab.Time(end)*ones(obj.numVals, 1); 
            d      = tab.Distance(end)*ones(obj.numVals, 1);
            v      = tab.Velocity(end)*ones(obj.numVals, 1);
            a      = zeros(obj.numVals, 1);
            pUse   = zeros(obj.numVals, 1);
            thrust = zeros(obj.numVals, 1); 
            q      = zeros(obj.numVals, 1); 
            CL     = zeros(obj.numVals, 1); 
            CD     = zeros(obj.numVals, 1); 
            L      = zeros(obj.numVals, 1); 
            D      = zeros(obj.numVals, 1); 
            LD     = zeros(obj.numVals, 1); 
            hDot   = zeros(obj.numVals, 1);                                     
            h      = tab.Altitude(end)*ones(obj.numVals, 1);                         
            Energy = tab.Energy(end)*ones(obj.numVals, 1);                      


            % Discretization on heading
            psi = linspace(0, dPsi, obj.numVals);


            % Sustained
            for i = 1:obj.numVals
                q(i) = 0.5*rho*v(i)^2;
                L(i) = lf*mass*g;
                aCentr = g*sqrt(lf^2 - 1);
                r = v(i)^2 / aCentr;
                CL(i) = L(i)/(Sref*q(i));
                CD(i) = CD0 + k*CL(i);
                LD(i) = CL(i)/CD(i);
                D(i) = L(i)/LD(i);
                thrust(i) = D(i);                                           
                propPower = thrust(i)*v(i);                                 % Check with propulsion model
                pUse = PowerCalc(propPower);                                % Check with propulsion model
                if(i>1)
                    d(i) = d(i-1)+(psi(i)-psi(i-1))*r;                      % Small angle approximation
                    t(i) = t(i-1)+(psi(i)-psi(i-1))/sqrt(aCentr/r);         
                    Energy(i) = Energy(i-1) + pUse*(t(i) - t(i-1));
                end
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
            Snew.Mass   = mass*ones(obj.numVals,1);
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

