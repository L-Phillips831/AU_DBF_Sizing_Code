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

        

        
    end
end

