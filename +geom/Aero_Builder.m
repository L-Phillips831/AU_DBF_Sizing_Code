classdef Aero_Builder
    % The Aero class is used for constructing a preliminary aerodynamic
    % buildup by component according to Raymer's method. The main outputs
    % of this class are the drag polar and CL_max of the geometry which are
    % then used for energy absorbtion analysis.
    
    properties
        vehicle (1,1) vehicle  % Vehicle object containing aircraft information
    end
    
    methods (Access = public)
        % Default constructor of the Aero class.
        % Params:
        %   - vehicle: the vehicle object containing the important
        %              information about the current design iteration
        %
        function obj = Aero_Builder(vehicle_)
            obj.vehicle =  vehicle_;
           
        end

    end
end

