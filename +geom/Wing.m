classdef Wing < geom.Lifting_Surface
    %WEIGHT_BUILDER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        end_geom (1,1) boolean

    end
    
    methods
        function obj = Wing(S_, AR_, span_, taper_, sweep_)
            obj = Lifting_Surface(S_, AR_, span_, taper_, sweep_);
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

