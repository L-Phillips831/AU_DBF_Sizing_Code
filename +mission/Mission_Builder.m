classdef Mission_Builder < handle  

    properties (Access = private)
         n_laps (1,1) uint16                       % Total number of laps to be flown in the mission.
         vehicle Vehicle                           % Vehicle object containing aircraft information.
         segments mission.Mission_Segment          % Heterogenous array of mission segment objects
         num_timesteps (1,1) double                % Number of timesteps for the mission segments to be split into
         vWind_NED (1,3) double

    end

    methods (Access = public)

      
        function obj = Mission_Builder(n_laps_, vehicle_, num_timesteps_, vWind_NED_, flag_)
            % Default Constructor for the mission class.
            % Params:
            %   - n_laps: positive integer that is the total number of laps
            %             flown in the respective mission.
            %   - vehicle: vehicle object containing relevant informataion of
            %              the current aircraft sizing and geometry.
            %

            obj.n_laps = n_laps_;
            obj.vehicle = vehicle_;
            obj.num_timesteps = num_timesteps_;
            obj.vWind_NED = vWind_NED_;

            switch flag_
                case 'mission 1'
                    obj.vehicle.mass = obj.vehicle.mission_1_W / Vehicle.g;
                case 'mission 2'
                    obj.vehicle.mass = obj.vehicle.mission_2_W / Vehicle.g;
                case 'mission 3'
                    obj.vehicle.mass = obj.vehicle.mission_3_W / Vehicle.g;
                otherwise
                    error('Improper mission flag.');
            end

        end

        
        function [t_total, E_total, tab] = run(obj, varargin)
            % Method for simulating the current mission.
            % Params:
            %   - tab (optional).
            % Returns:
            %   - t_total: the total time it took to complete the mission. Used
            %              in scoring analysis.
            %   - E_total: the total estimated energy usage during the mission.
            %              Used to verify geometry convergence
            %

            flight_keys = ["Airspeed_NED","Groundspeed_NED","Windspeed_NED","Eulers","Position_NED", ...
               "Mass","Throttle","Thrust_Body","q","CL","CD","Alpha","Gamma", ...
               "Energy","Power","Time"];

            % If tbl is not created yet, create it.
            if isempty(varargin)
                tab = table('Size',[0 numel(flight_keys)], ...
                    'VariableNames', flight_keys, ...
                    'VariableTypes', repmat("double",1,numel(flight_keys)));

            elseif isscalar(varargin)
                tab = varargin{1};
            else 
                error("Too many inputs. Either input table if it exists or no arguments.");
            end

            % Propagate the segments
            for segment = obj.segments
                tab = segment.run(tab);
                
            end


            t_total = tab.Time(end);
            E_total = tab.Energy(end);

        end

        function obj = clear(obj)
            % Method to clear the mission segment list. Used for repeated
            % section analaysis to prevent re-solving previous segments
            % Params:
            %   - current mission_builder object
            % Returns:
            %   - the updated object
            %
            
            obj.segments = mission.Mission_Segment.empty();
        end


        
        function obj = add_takeoff(obj, T_set, T_set_climb)
            % Method for adding takeoff mission segment
            % Params:
            %   - 
            %

            takeoff_ = mission.Takeoff(T_set, T_set_climb, obj.num_timesteps, obj.vehicle, obj.vWind_NED);
            obj.segments(end+1) = takeoff_;

            

        end


        function obj = add_climb(obj, hEnd, T_set)
            % Method for adding climb mission segment
            % Params:
            %   - hEnd: final height after climb [m]
            %   - vClimb: climb airspeed [m/s]
            %   - hDotClimb: climb rate [m/s]
            % Returns:
            %   - updated Mission_Builder object
            %

            climb_segment = mission.Climb(hEnd, T_set, obj.num_timesteps, obj.vehicle);
            obj.segments(end+1) = climb_segment;

        end


        
        function obj = add_cruise(obj, dDelta, T_set)
            % Method for adding cruise mission segment
            % Params:
            %   - dDelta: distance traveled during the segment [m]
            %   - T_set: throttle setting for the mission [-]
            % Returns:
            %   - updated Mission_Builder object
            %

            cruise_segment = mission.Cruise(dDelta, T_set, obj.num_timesteps, obj.vehicle);
            obj.segments(end+1) = cruise_segment;

        end


        function obj = add_turn(obj, T_set, dPhi, option)
            % Method for adding turn mission segment
            % Params:
            %   - dPhi: heading change [deg]
            %   - option: specification of instantaneous or sustained turn
            % Returns:
            %   - updated Mission_Builder Object
            %

            turn_segment = mission.Turn(T_set, dPhi, obj.num_timesteps, obj.vehicle, option);
            obj.segments(end+1) = turn_segment;

        end


    end
end