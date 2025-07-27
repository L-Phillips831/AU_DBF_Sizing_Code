classdef (Abstract) Mission_Segment
    % Abstract base class for defining mission segments. This abstract
    % class forces inheriting classes to define the flight state table as
    % well as a run method to update the table. Both the table property and
    % run method are required for use in the mission builder
    
    properties (Abstract)
        table double             % Running table with flight state properties
    end
    
    methods (Abstract)

        % Abstract run method that returns updated table
        table = run(obj, varagin)

    end
end

