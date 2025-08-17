classdef Turn < Mission_Segment
    %TURN Summary of this class goes here
    %   Detailed explanation goes here
    
    properties       
        table double
    end
    
    methods

        function obj = Turn(table)

            obj.table = table;
        end

        function table = run(obj, option)
            
            switch option
                case {'instantaneous', 'Instantaneous'}
                    obj = obj.InstantaneousTurn();

                case {'sustained', 'Sustained'}
                    obj = obj.SustainedTurn();

                otherwise  
                    error("Option must be (I/i)nstantaneous or (S/s)ustained");

            end

            table = obj.table;
        end
        
        
        function MissionTable = InstantaneousTurn(obj, dPhi)
        % Instantaneous Turn Mission Segment

        % Description: Takes heading change and a flight history table as
        % inputs. Finds the minimum lift bound between structural and
        % aerodynamic maximum lift available. Gets centripetal acceleration
        % as Lift/mass. Gets thrust and power from throttle setting and
        % velocity. Gets forward acceleration from (T-D)/mass. Turn radius
        % not fixed because speed and lift vary.

        % General variables
            tab  = obj.MissionTable;                                        % Need call
            Sref = obj.Sref;                                                % Need call
            k    = obj.k;                                                   % Need call
            CD0  = obj.CD0;                                                 % Need call
            rho  = obj.rho;                                                 % Need call
            mass = tab.mass(end);                                           % Need call
            g    = obj.g;                                                   % Need call

        % Instantaneous variables
            lfStruc = obj.lfStruc;                                          % Need call, maximum structural load factor


        % Initialize table variables
            t      = tab.Time(end)*ones(numVals, 1); 
            d      = tab.Distance(end)*ones(numVals, 1);
            v      = tab.Velocity(end)*ones(numVals, 1);
            a      = zeros(numVals, 1);
            pUse   = zeros(numVals, 1);
            thrust = zeros(numVals, 1); 
            q      = zeros(numVals, 1); 
            CL     = zeros(numVals, 1); 
            CD     = zeros(numVals, 1); 
            L      = zeros(numVals, 1); 
            D      = zeros(numVals, 1); 
            LD     = zeros(numVals, 1); 
            hDot   = zeros(numVals, 1);                                     
            h      = tab.Altitude(end)*ones(numVals, 1);                         
            Energy = tab.Energy(end)*ones(numVals, 1);                      


        % Discretization on heading
            phi = linspace(0, dPhi, numVals);
           

        % Instantaneous
            for i = 1:numVals
                if i>1
                    v(i) = v(i-1) + a(i-1)*dt;
                end
                q(i) = 0.5*rho*v(i)^2;
                LmaxAero = CLmax*q(i)*Sref;
                LmaxStructure = lfStruc*mass*g;
                L(i) = min([LmaxAero, LmaxStructure]);                      % Constrained by aero or structure
                aCentr = L(i)/mass;                                         % bank at 90 deg
                CL(i) = L(i)/(Sref*q(i));
                CD(i) = CD0 + k*CL(i)^2;
                LD(i) = CL(i)/CD(i);
                D(i) = L(i)/LD(i);
                [thrust(i), pUse(i)] = PropulsionCalc(v(i), T_set);         % Need function
                a(i) = (thrust(i) - D(i))/mass;                             % forward acceleration
                r = v(i)^2 / aCentr;
                omega = v(i)/r;                                             % turn rate in rad/s
                if i>1
                    Energy(i) = Energy(i-1) + pUse*dt;
                    dt = (phi(i)-phi(i-1))/omega;
                    t(i) = t(i-1) + dt;
                    d(i) = d(i-1) + (phi(i) - phi(i-1))*r;                  % Small angle approx
                    hDot(i) = g*(t(i) - t(1));
                    h(i) = h(1) - 0.5*g*(t(i) - t(1));                      
                end                
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



        function MissionTable = SustainedTurn(obj, dPhi)
            % Sustained Turn Mission Segment

            % Description: Takes heading change and the aircraft object.
            % Aircraft object contains a table of flight time history.
            % Gets lift from the sustained load factor. Centripetal
            % acceleration is found from g and the sustained lf. Thrust
            % equals drag and propPower = Thrust*v. Gets pUse from the
            % propulsion model with input propPower.

            % General variables
            tab  = obj.MissionTable;                                        % Need call
            Sref = obj.Sref;                                                % Need call
            k    = obj.k;                                                   % Need call
            CD0  = obj.CD0;                                                 % Need call
            rho  = obj.rho;                                                 % Need call
            mass = tab.mass(end);                                           % Need call
            g    = obj.g;                                                   % Need call
            lf   = obj.lfSust;                                              % Need call, sustained load factor


            % Initialize table variables
            t      = tab.Time(end)*ones(numVals, 1); 
            d      = tab.Distance(end)*ones(numVals, 1);
            v      = tab.Velocity(end)*ones(numVals, 1);
            a      = zeros(numVals, 1);
            pUse   = zeros(numVals, 1);
            thrust = zeros(numVals, 1); 
            q      = zeros(numVals, 1); 
            CL     = zeros(numVals, 1); 
            CD     = zeros(numVals, 1); 
            L      = zeros(numVals, 1); 
            D      = zeros(numVals, 1); 
            LD     = zeros(numVals, 1); 
            hDot   = zeros(numVals, 1);                                     
            h      = tab.Altitude(end)*ones(numVals, 1);                         
            Energy = tab.Energy(end)*ones(numVals, 1);                      


            % Discretization on heading
            phi = linspace(0, dPhi, numVals);


            % Sustained
            for i = 1:numVals
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
                    d(i) = d(i-1)+(phi(i)-phi(i-1))*r;                      % Small angle approximation
                    t(i) = t(i-1)+(phi(i)-phi(i-1))/sqrt(aCentr/r);         
                    Energy(i) = Energy(i-1) + pUse*(t(i) - t(i-1));
                end
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

