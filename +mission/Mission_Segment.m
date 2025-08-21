classdef (Abstract) Mission_Segment < matlab.mixin.Heterogeneous
    % Abstract base class for defining mission segments. This abstract
    % class forces inheriting classes to define the vehicle as
    % well as a run method to update the table. Both the table property and
    % run method are required for use in the mission builder
    
    
    methods (Abstract)

        % Abstract run method that returns updated table.
        tbl = run(obj, tbl, varargin)

    end
end

