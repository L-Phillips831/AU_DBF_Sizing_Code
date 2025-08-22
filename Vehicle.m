classdef Vehicle
    % The vehicle class is the output class of the MOG sizing codebase
    % that contains all pertinant information regarding the current
    % iteration of the design. Multiple vehicle objects will be created by
    % the optimizer in order to maximize mission performance
    
    properties
        components struct                           % struct of components
        % Propulsion (1,1) powerplant.Propulsion

        mass (1,1) double
        Cl_alpha (1,1) double
        CM_alpha (1,1) double
        CD_0 (1,1) double
        K1 (1,1) double
        K2 (1,1) double

    end
    
    methods

        % Default constructor of the Vehicle class.
        % Params:
        %   - 
        %
        function obj = Vehicle(params, constraints)

        end

        % Method to get the Cl_alpha of the entire aircraft. Serves as a
        % wrapper function for avl related methods.
        % Params:
        %   - none
        % Returns:
        %   - updated vehicle object
        %
        function obj = get_Cl_alpha(obj)

        end
        
    end

    methods (Access = private)

        function filepath = generate_geom_file(obj)

        end

        function [filepath, num_cases] = generate_case_file(alpha_range)

        end

        function filepath = generate_cmd_file(geom_file, case_file, num_cases, results_file)

        end

        function obj = parse_avl(results_path)

        end

    end

    methods (Static)

        function results = run_avl(cmd_file)

        end

    end

    

end

