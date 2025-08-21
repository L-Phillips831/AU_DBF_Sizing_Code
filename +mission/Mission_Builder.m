classdef Mission_Builder < handle  % stupid in-place modification requirement

    properties (Access = private)
         n_laps (1,1) uint16                       % Total number of laps to be flown in the mission.
         vehicle (1,1) Vehicle                     % Vehicle object containing aircraft information.
         segments mission.Mission_Segment          % Heterogenous array of mission segment objects
         num_timesteps (1,1) double                % Number of timesteps for the mission segments to be split into
    
    end

    methods (Access = public)

        % Default Constructor for the mission class.
        % Params:
        %   - n_laps: positive integer that is the total number of laps
        %             flown in the respective mission.
        %   - vehicle: vehicle object containing relevant informataion of
        %              the current aircraft sizing and geometry.
        %
        function obj = Mission_Builder(n_laps_, vehicle_, num_timesteps_)
            obj.n_laps = n_laps_;
            obj.vehicle = vehicle_;
            obj.num_timesteps = num_timesteps_;

        end

        % Method for simulating the current mission.
        % Params:
        %   - none.
        % Returns:
        %   - t_total: the total time it took to complete the mission. Used
        %              in scoring analysis.
        %   - E_total: the total estimated energy usage during the mission.
        %              Used to verify geometry convergence
        %
        function [t_total, E_total] = run(obj)

            flight_keys = ["Time","E","Power","Distance","V","Acceleration","Altitude","hDot", ...
               "Mass","Thrust","q","CL","CD","Lift","Drag","LD"];

            tab = table('Size',[0 numel(flight_keys)], ...
            'VariableNames',flight_keys, ...
            'VariableTypes',"double");

            for segment = obj.segments
                tab = segment.run(tab);
                
            end


            t_total = tab.Time(end);
            E_total = tab.E(end);

        end


        % Method for adding takeoff mission segment
        % Params:
        %   - 
        %
        function obj = add_takeoff(obj)

        end


        % Method for adding climb mission segment
        % Params:
        %   - hEnd: final height after climb [m]
        %   - vClimb: climb airspeed [m/s]
        %   - hDotClimb: climb rate [m/s]
        % Returns:
        %   - updated Mission_Builder object
        %
        function obj = add_climb(obj, hEnd, vClimb, hDotClimb)

            climb_segment = mission.Climb(hEnd, vClimb, hDotClimb, obj.num_timesteps, obj.vehicle);
            obj.segments(end+1) = climb_segment;

        end


        % Method for adding cruise mission segment
        % Params:
        %   - dDelta: distance traveled during the segment [m]
        %   - T_set: throttle setting for the mission [-]
        % Returns:
        %   - updated Mission_Builder object
        %
        function obj = add_cruise(obj, dDelta, T_set)

            cruise_segment = mission.Cruise(dDelta, T_set, obj.num_timesteps, obj.vehicle);
            obj.segments(end+1) = cruise_segment;

        end

        % Method for adding turn mission segment
        % Params:
        %   - dPhi: heading change [deg]
        %   - option: specification of instantaneous or sustained turn
        % Returns:
        %   - updated Mission_Builder Object
        %
        function obj = add_turn(obj, dPhi, option)

            turn_segment = mission.Turn(dPhi, obj.num_timesteps, obj.vehicle, option);
            obj.segments(end+1) = turn_segment;

        end


    end
end