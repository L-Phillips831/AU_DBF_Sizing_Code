classdef Vehicle
    % The vehicle class is the output class of the MOG sizing codebase
    % that contains all pertinant information regarding the current
    % iteration of the design. Multiple vehicle objects will be created by
    % the optimizer in order to maximize mission performance
    
    properties
        name string
        components geom.Component                       % struct of components
        Propulsion (1,1) powerplant.Propulsion
        CG (3,1) double                                 % X,Y,Z loc of CG

        S_ref (1,1) double
        c_ref (1,1) double
        b_ref (1,1) double

        mass (1,1) double
        Cl_alpha (1,1) double
        CM_alpha (1,1) double
        CD_0 (1,1) double
        K1 (1,1) double
        K2 (1,1) double

    end
    
    methods

        % Default constructor of the Vehicle class.
        % Params:
        %   - 
        %
        function obj = Vehicle(params, constraints)

        end
        

        function obj = add_wing()

        end


        function obj = add_tail()

        end


        % Method to get the Cl_alpha of the entire aircraft. Serves as a
        % wrapper function for avl related methods.
        % Params:
        %   - none
        % Returns:
        %   - updated vehicle object
        %
        function obj = get_Cl_alpha(obj)

        end
        
    end

    methods (Access = private)


        % Method to generate the complete geom input file for AVL analysis.
        % Loops through each user added component and adds its
        % corresponding surface into the file.
        % Params:
        %   - current object
        % Returns:
        %   - none
        %
        function generate_geom_file(obj)

            file_path = "+lib\+AVL\Input\geom_file.avl";

            if exist(file_path, 'file')
                delete(file_path)
            end


            str = [
                    "#Mach",...
                    "0.0",...    
                    "#IYsym   IZsym   Zsym",...
                    "0       0       0.0",...
                    "#Sref    Cref    Bref",...
                    sprintf("%.1f %.1f %.1f", obj.S_ref, obj.c_ref,  obj.b_ref),...
                    "#Xref    Yref    Zref",...
                    sprintf("%.1f %.1f %.1f", obj.CG(1), obj.CG(2), obj.CG(3)),...
                    ];

            for component = obj.components
                str = [str, component.convert_to_avl_geom()]; 
            end

            writelines(str, file_path);

        end



        function num_cases = generate_case_file(alpha_range, airspeed, file_path)

            file_path = "+lib\+AVL\Input\case_file.run";
    
            if exist(file_path, 'file')
                delete(file_path)
            end


            num_cases = numel(alpha_range);









            writelines(str, file_path);

        end

        function saved_files = generate_cmd_file(geom_file, case_file, num_cases, results_file)
            
            file_path = "+lib\+AVL\Input\cmd_file.cmd";
    
            if exist(file_path, 'file')
                delete(file_path)
            end









            writelines(str, file_path);

        end





        function obj = parse_avl(saved_files)

        end

    end

    methods (Static)


        function run_avl(cmd_file)

            system(sprintf('cd "+lib\+AVL" && avl < "%s"', cmd_file)); 

        end

    end

    

end

