classdef (Abstract) Mission_Segment
    % Abstract base class for defining mission segments. This abstract
    % class forces inheriting classes to define the vehicle as
    % well as a run method to update the table. Both the table property and
    % run method are required for use in the mission builder
    
    properties (Abstract)
        vehicle (1,1) Vehicle

    end
    
    methods (Abstract)

        % Abstract run method that returns updated table.
        table = run(obj, table, varagin)

    end
end

