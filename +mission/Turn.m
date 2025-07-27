classdef Turn < Mission_Segment
    %TURN Summary of this class goes here
    %   Detailed explanation goes here
    
    properties       
        table double
    end
    
    methods
        function obj = Turn(table)

            obj.table = table;
        end

        function table = run(obj, option)
            
            switch option
                case {'instantaneous', 'Instantaneous'}
                    obj = obj.InstantaneousTurn();

                case {'sustained', 'Sustained'}
                    obj = obj.SustainedTurn();

                otherwise  
                    error("Option must be (I/i)nstantaneous or (S/s)ustained");

            end

            table = obj.table;
        end
        
        function obj = InstantaneousTurn(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            obj.table = obj.Property1 + inputArg;
        end

        function obj = SustainedTurn(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            obj.table = obj.Property1 + inputArg;
        end
    end
end

