classdef Wing < Lifting_Surface
    %WEIGHT_BUILDER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        end_geom (1,1) boolean
    end
    
    methods
        function obj = Wing(inputArg1,inputArg2)
            %WEIGHT_BUILDER Construct an instance of this class
            %   Detailed explanation goes here
            obj.Property1 = inputArg1 + inputArg2;
        end
        
        function cl_alpha = get_wing_cl_alpha(obj,M)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here


            % Effect of wing ending geom
            if isfield(obj.geom, 'EndGeom')
                endplate = obj.geom.EndGeom;
                if endplate.style == "Endplate"
                    AR_ = AR_*(1 + 1.9*endplate.h/endplate.b);

                elseif endplate.style == "Winglet"
                    AR_ = AR_*(1 + endplate.h/endplate.b)^2;
                end

            end

            
            
        end
    end
end

