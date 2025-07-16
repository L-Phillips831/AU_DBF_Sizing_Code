classdef Aero_Builder
    % The Aero class is used for constructing a preliminary aerodynamic
    % buildup by component according to Raymer's method. The main outputs
    % of this class are the drag polar and CL_max of the geometry which are
    % then used for energy absorbtion analysis.
    
    properties
        geom (1,1) struct  % Geometry struct containing sizing info for the geom
    end
    
    methods (Access = public)
        % Default constructor of the Aero class.
        % Params:
        %   - vehicle: the vehicle object containing the important
        %              information about the current design iteration
        %
        function obj = Aero_Builder(geom_)
            obj.geom =  geom_;
           
        end

        function CL_alpha = get_CL_alpha_sub(obj, M)

            AR_ = obj.geom.AR;
            M_ = M;
            eta_ = 0.95;        % Approximation of lift curve slope w/ M recommended by Raymer.
            lambda_max_t = 0;   % Assuming rectangular geometry with no sweep

            % Effect of wing ending geom
            if isfield(obj.geom, 'EndGeom')
                endplate = obj.geom.EndGeom;
                if endplate.style == "Endplate"
                    AR_ = AR_*(1 + 1.9*endplate.h/endplate.b);

                elseif endplate.style == "Winglet"
                    AR_ = AR_*(1 + endplate.h/endplate.b)^2;
                end

            end

            Beta = sqrt(1-M_^2);
            fuse_effects = 0.98;  % Raymer estimation of fuselage effects
            CL_alpha =  2*pi*AR_ / (2 + sqrt(4 + AR_^2*Beta^2 / eta_^2 * (1 + tan(lambda_max_t)^2/Beta^2))) * fuse_effects;


        end



    end
end

