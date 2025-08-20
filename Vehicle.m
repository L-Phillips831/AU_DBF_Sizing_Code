classdef Vehicle
    % The vehicle class is the parent class of the EAGLE sizing codebase
    % that contains all pertinant information regarding the current
    % iteration of the design. Multiple vehicle objects will be created by
    % the optimizer in order to maximize mission performance
    
    properties
        components struct                           % struct of components
        Propulsion (1,1) powerplant.Propulsion
        
    end

    properties (Access = private)
        Weight (1,1) double
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

        % Method for retrieving aircraft weight.
        % Params:
        %   - None
        % Returns:
        %   - Aircraft Weight
        function weight = get_vehicle_weight(obj)

            if ~isempty(obj.Weight)
                weight = obj.Weight;
            else 
                error("Aircraft Weight has not been determined");
            end

        end
        
    end
end

