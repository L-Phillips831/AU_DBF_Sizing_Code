classdef Vehicle
    % The vehicle class is the parent class of the EAGLE sizing codebase
    % that contains all pertinant information regarding the current
    % iteration of the design. Multiple vehicle objects will be created by
    % the optimizer in order to maximize mission performance
    
    properties
        components struct                           % struct of components
        Propulsion (1,1) powerplant.Propulsion
        
    end
    
    methods

        % Default constructor of the Vehicle class.
        % Params:
        %   - 
        %
        function obj = Vehicle()

        end
        
    end
end

