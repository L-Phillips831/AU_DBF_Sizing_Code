classdef Vehicle
    % The vehicle class is the output class of the MOG sizing codebase
    % that contains all pertinant information regarding the current
    % iteration of the design. Multiple vehicle objects will be created by
    % the optimizer in order to maximize mission performance
    
    properties
        name string                                     % Vehicle Name
        components geom.Component                       % struct of components
        Propulsion (1,1) powerplant.Propulsion          % Propulsion system
        CG (3,1) double                                 % X,Y,Z loc of CG [m]

        S_ref (1,1) double                              % Vehicle reference area  [m]
        c_ref (1,1) double                              % Vehicle reference chord [m]
        b_ref (1,1) double                              % Vehicle reference span  [m]

        mass (1,1) double                               % Vehicle mass  [kg]
        Cl_alpha (1,1) double                           % Vehicle Cl_alpha [-]
        CM_alpha (1,1) double                           % Vehicle CM_alpha [-]
        SM (1,1) double                                 % Vehicle Static Margin [-]
        CD_0 (1,1) double                               % Vehicle zero-lift drag coefficient [-]
        K2 (1,1) double                                 % Vehicle parabolic drag coefficient [-]
        K1 (1,1) double                                 % Vehicle linear drag coefficient [-]
    
    end

    properties (Access = private)

        geom_file = "+lib\+AVL\Input\geom_file.avl";
        case_file = "+lib\+AVL\Input\case_file.run";
        cmd_file = "+lib\+AVL\Input\cmd_file.cmd";
        saves_dir =  "+lib\+AVL\Saves";

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


        % Method to complete analysis of the entire aircraft. Serves as a
        % wrapper function for avl related methods and component based 
        % weight buildup.
        % Params:
        %   - none
        % Returns:
        %   - updated vehicle object
        %
        function obj = analyze_airframe(obj)

            % Weight Buildup
            for component = obj.components
                obj.mass = obj.mass + component.get_mass();
            end

            % Aero Buildup
            alpha_range = -6:2:20;
            obj.generate_geom_file();
            obj.generate_case_file(alpha_range);
            saved_files = obj.generate_cmd_file();
            Vehicle.run_avl(obj.cmd_file);
            obj.parse_avl(saved_files);

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

            if exist(obj.geom_file, 'file')
                delete(obj.geom_file)
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

            writelines(str, obj.geom_file);

        end


        % Method to generate case file for running alpha sweep in AVL.
        % Params:
        %   - current object
        %   - alpha_range: Double array of aoa values
        % Returns:
        %   - the number of cases generated
        %
        function num_cases = generate_case_file(obj, alpha_range)

    
            if exist(obj.case_file, 'file')
                delete(obj.case_file)
            end


            num_cases = numel(alpha_range);

            str = "";

            for i = 1:num_cases
              new_str = [
                         "----------------------------------------------",...
                         sprintf("Run case %f: - unnamed", i),...
                         "",...
                         sprintf("alpha        ->  alpha       =   %.5f", alpha_range(i)),...
                         "beta         ->  beta        =   0.00000",...    
                         "pb/2V        ->  pb/2V       =   0.00000",...    
                         "qc/2V        ->  qc/2V       =   0.00000",...    
                         "rb/2V        ->  rb/2V       =   0.00000",...    
                         "flap         ->  flap        =   0.00000",...    
                         "aileron      ->  aileron     =   0.00000",...    
                         "elevator     ->  elevator    =   0.00000",...    
                         "rudder       ->  rudder      =   0.00000",...    
                            ];

              str = [str, new_str];
            end


            writelines(str, obj.case_file);

        end

        % Method to generate the keystroke commands to be fed to avl for
        % running all test cases and saving to separate files.
        % Params:
        %   - the current object
        %   - num_cases: Double number of cases to be ran
        % Returns:
        %   - saved_files: str array of filepaths containing solved data
        %
        function saved_files = generate_cmd_file(obj, num_cases)
            
    
            if exist(obj.cmd_file, 'file')
                delete(obj.cmd_file)
            end


            str = [
                   sprintf("LOAD %s", obj.geom_file),...
                   sprintf("CASE %s", obj.case_file),...
                   "OPER",...
            ];

            saved_files = [];

            for i = 1:num_cases

                current_save = fullfile(obj.saves_dir, sprintf("Case_%s.txt", i));

                if exist(current_save, 'file')
                    delete(current_save);
                end

                saved_files = [saved_files, current_save];

                new_str = [
                            "X",...
                            sprintf("ST %s", current_save),... 
                            ];

                str = [str, new_str];
            end

            str = [str, "QUIT"];

            writelines(str, obj.cmd_file);

        end




        % Method for parsing avl solved data and updating object properties 
        % with solved aero data.
        % Params:
        %   - saved_files: str array of filepaths to saved files
        % Returns:
        %   - Updated object with aero data
        %
        function obj = parse_avl(obj, saved_files)

            alpha_arr = zeros(numel(saved_files), 1);
            CL_arr    = zeros(numel(saved_files), 1);
            Cm_arr    = zeros(numel(saved_files), 1);
            CD_arr    = zeros(numel(saved_files), 1);



            getValue = @(text, keyword) str2double( ...
                    regexp(text, keyword + "\s*=?\s*([-\d\.Ee]+)", 'tokens', 'once') ...
                );

            for f = 1:numel(saved_files)
                lines = readlines(saved_files(f));
                text = strjoin(lines, " ");

                

                alpha_arr(f) = getValue(text, "Alpha");
                CL_arr(f)    = getValue(text, "CLtot");
                Cm_arr(f)    = getValue(text, "Cmtot");
                CD_arr(f)    = getValue(text, "CDtot");
            end

        
        [alpha_arr, sort_idx] = sort(alpha_arr);
        CL_arr = sort(sort_idx);
        Cm_arr = sort(sort_idx);
        CD_arr = sort(sort_idx);

        Cl_fit = polyfit(alpha_arr, CL_arr, 1); obj.Cl_alpha = Cl_fit(1);
        CM_fit = polyfit(alpha_arr, Cm_arr, 1); obj.CM_alpha = CM_fit(1);
        CD_fit = polyfit(CL_arr, CD_arr, 2); 
        obj.K2 = CD_fit(1); obj.K1 = CD_fit(2); obj.CD_0 = CD_fit(3); 

        obj.SM = -obj.CM_alpha / obj.Cl_alpha;
        
        end

    end

    methods (Static)


        function run_avl(cmd_file)

            system(sprintf('cd "+lib\+AVL" && avl < "%s"', cmd_file)); 

        end

    end

    

end

