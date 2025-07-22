classdef Turn < Mission_Segment
    %TURN Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
       
        Property1
    end
    
    methods
        function obj = Turn(inputArg1,inputArg2)
            %TURN Construct an instance of this class
            %   Detailed explanation goes here
            obj.Property1 = inputArg1 + inputArg2;
        end
        
        function outputArg = InstantaneousTurn(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end

        function outputArg = method2(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
    end
end

