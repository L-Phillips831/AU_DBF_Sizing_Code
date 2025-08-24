classdef Component < matlab.mixin.Heterogeneous
    % Superclass definition for component objects. Abstract class 
    % forces creation of weight buildup and aero buildup methods in
    % implementing classes
    
    properties (Abstract)
        
    end
    
    methods (Abstract)
        
        mass = get_mass(obj, varargin)

    end
end

