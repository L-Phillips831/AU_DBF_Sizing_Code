classdef Lifting_Surface < Component & handle
    % The Aero class is used for constructing a preliminary aerodynamic
    % buildup by component according to Raymer's method. The main outputs
    % of this class are the drag polar and CL_max of the geometry which are
    % then used for energy absorbtion analysis.
    
    properties
        name string
        S (1,1) double
        AR (1,1) double
        span (1,1) double
        chord (1,1) double
        taper (1,1) double
        sweep (1,1) double
        x_loc (1,1) double
        z_loc (1,1) double
        y_loc (1,1) double
        rotation (1,1) double

        airfoil_str string
        Cl_alpha (1,1) Double
        Cl_max (1,1) Double

    end
    
    methods (Access = public)
        % Default constructor of the Lifting surface class.
        % Params:
        %   - S_: Double reference area
        %   - AR_: Double aspect ratio
        %   - span_: Double span
        %   - taper_: Double taper ratio
        %   - sweep_: Double sweep
        %   - airfoil_: String name of airfoil
        % Returns:
        %   - updated object
        %
        function obj = Lifting_Surface(name_, S_, AR_, span_, taper_, sweep_, airfoil_)
           obj.name = name_;
            obj.S = S_;
           obj.AR = AR_;
           obj.span = span_;
           obj.taper = taper_;
           obj.sweep = sweep_;
           obj.airfoil_str = airfoil_;
           
        end


        % Method to set the placement and orientation of a lifting surface. Used for
        % aero buildup and stability calculations. X loc is measuered
        % relative to datum, y and z loc measured relative to fuselage centerline
        % rotation relative to x-y plane and rotating about x axis
        % Params:
        %   - x_loc_: Double x location
        %   - y_loc_: Double y location
        %   - z_loc_: Double z location
        %   - rot_: Double rotation angle
        %  Returns:
        %   - updated object
        %
        function obj = place_surface(x_loc_, y_loc_, z_loc_, rot_)
            obj.x_loc = x_loc_;
            obj.z_loc = z_loc_;
            obj.y_loc = y_loc_;
            obj.rotation = rot_;
                
        end

        % Method to convert the object properties to the section that would
        % appear in an AVL geometry input file. Lines are separated as
        % elements in a string array.
        % Params:
        %   - current obj
        % Returns:
        %   - str
        %
        function str = convert_to_avl_geom(obj)

            root_chord = obj.chord;
            tip_chord = root_chord * obj.taper;

            str = [
                   "#====================================================================",...
                   "SURFACE",...
                   obj.name,...
                   "#Nchordwise  Cspace   Nspanwise   Sspace",...
                   "8            1.0       12         1.0",...
                   "#",...
                   "YDUPLICATE",...
                   "0.0",...
                   "#",...
                   "TRANSLATE",...
                   sprintf("%.1f %.1f %.1f", obj.x_loc, obj.y_loc, obj.z_loc),...
                   "#-------------------------------------------------------------",...
                   "SECTION",...
                   "#Xle    Yle    Zle     Chord   Ainc  Nspanwise  Sspace",...
                   sprintf("%.1f %.1f %.1f %.1f %.1f %.1f %.1f", 0, 0, 0, root_chord, 0, 0, 0),...
                   "AFILE",...
                   obj.airfoil_str,...
                   "#-------------------------------------------------------------",...
                   "SECTION",...
                   "#Xle    Yle    Zle     Chord   Ainc  Nspanwise  Sspace",...
                   sprintf("%.1f %.1f %.1f %.1f %.1f %.1f %.1f", 0, obj.span/2, 0, tip_chord, 0, 0, 0),...
                   "AFILE",...
                   obj.airfoil_str,...
                   ];

        end

    end

end

