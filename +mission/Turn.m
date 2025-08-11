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
        
        
        function MissionTable = InstantaneousTurn(obj)
        % Instantaneous Turn Mission Segment

        % Description: Takes heading change, a flight history table, and a
        % path label between instantaneous and sustained turns.
        % How are parameters defined for instantaneous or sustained?
        % Sustained -> Constant turn rate at a certain load factor.
        % Instantaneous -> Set throttle and limited by aero/structure.

        % General variables
            tab  = obj.MissionTable;                                        % Need call
            Sref = obj.Sref;                                                % Need call
            k    = obj.k;                                                   % Need call
            CD0  = obj.CD0;                                                 % Need call
            rho  = obj.rho;                                                 % Need call
            mass = tab.mass(end);                                           % Need call
            g    = obj.g;                                                   % Need call

        % Instantaneous variables
            lfMax = 0;                                                      % Need call, maximum structural load factor


        % Initialize table variables
            t      = tab.Time(end)*ones(numVals, 1); 
            d      = tab.Distance*ones(numVals, 1);
            v      = tab.Velocity*ones(numVals, 1);
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
            h      = tab.Altitude*ones(numVals, 1);                         
            Energy = tab.Energy(end)*ones(numVals, 1);                      


        % Discretization on heading
            phi = linspace(0, dPhi, numVals);
           

        % Instantaneous
            for i = 1:numVals
                if i>1
                    v(i) = v(i-1) + a(i-1)*dt;
                end
                % If i>1, calculate velocity
                q(i) = 0.5*rho*v(i)^2;
                LmaxAero = CLmax*q(i)*Sref;
                LmaxStructure = lfMax*mass*g;
                L(i) = min([LmaxAero, LmaxStructure]);                      % Constrained by aero or structure
                aCentr = L(i)/mass;                                         % bank at 90 deg)
                CL(i) = L(i)/(Sref*q(i));
                CD(i) = CD0 + k*CL(i)^2;
                LD(i) = CL(i)/CD(i);
                D(i) = L(i)/LD(i);
                [thrust(i), pUse(i)] = PropulsionCalc(v(i), T_set);         % Need function
                a(i) = (thrust(i) - D(i))/mass;
                r = v(i)^2 / aCentr;
                omega = v(i)/r;
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



        function MissionTable = SustainedTurn(obj)
            % Sustained Turn Mission Segment
            %   Detailed explanation goes here

            % General variables
            tab  = obj.MissionTable;                                        % Need call
            Sref = obj.Sref;                                                % Need call
            k    = obj.k;                                                   % Need call
            CD0  = obj.CD0;                                                 % Need call
            rho  = obj.rho;                                                 % Need call
            mass = tab.mass(end);                                           % Need call
            g    = obj.g;                                                   % Need call


            % Sustained variables
            lf = 0;                                                         % Need call, sustained load factor


            % Initialize table variables
            t      = tab.Time(end)*ones(numVals, 1); 
            d      = tab.Distance*ones(numVals, 1);
            v      = tab.Velocity*ones(numVals, 1);
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
            h      = tab.Altitude*ones(numVals, 1);                         
            Energy = tab.Energy(end)*ones(numVals, 1);                      


            % Discretization on heading
            phi = linspace(0, dPhi, numVals);


            % Sustained
            for i = 1:numVals
                q(i) = 0.5*rho*v(i)^2;
                L(i) = lf*mass*g;
                aCentr = sqrt(lf^2 - 1);
                r = v(i)^2 / aCentr;
                CL(i) = L(i)/(Sref*q(i));
                CD(i) = CD0 + k*CL(i);
                LD(i) = CL(i)/CD(i);
                D(i) = L(i)/LD(i);
                thrust(i) = D(i);                                           
                propPower = thrust(i)*v(i);                                 % Necessary?
                pUse = PowerCalc(propPower);                                % Need function. Want v and thrust?
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

