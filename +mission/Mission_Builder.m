classdef Mission_Builder

    properties (Access = private)
         n_laps (1,1) uint16                % Total number of laps to be flown in the mission.
         vehicle (1,1) Vehicle              % Vehicle object containing aircraft information.
         segments mission.Mission_Segment   % Heterogenous array of mission segment objects
         num_timesteps (1,1) double         % Number of timesteps for the mission segments to be split into
    
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
        %   - flag: boolean flag to ensure mission was completed without issue.
        %
        function [t_total, E_total, flag] = run(obj)

            for segment = obj.segments
                
            end

        end


        % Method to add mission segment to list of segments
        % Params:
        %   - segment: segment to be added
        % Returns:
        %   - obj: modified mission_builder obj
        %
        function obj = add_segment(obj, segment)
            obj.segments = [obj.segments, segment];

        end
    end
end