classdef EnironmentSetupSimulation
    % all units are in SI
    properties
        n_obstacles % number of obstacles
        obstacle_matrix %containing the obstacle matix
        ref_waypoints % define the goal point to head to
        origin %origin
        initial_zone
        end_zone
        environment_type
        static_goal % to test the static goal




    end

    properties (Access = private)
        workspace_dimension  % define the dimension of the worksapce
    end
    methods
        % --- constructor
        function obj = EnironmentSetupSimulation(environment_type, obs_pos)

            % Workspace dimensions are constant in all casess
            obj.workspace_dimension.length = 4.5;
            obj.workspace_dimension.width = 3.2;
            obj.origin = [0;0]; %centered in the rectangle
            obj.environment_type = environment_type;


            obj = obj.setupEnvironments(environment_type, obs_pos);
        end
        % ---- sets ups the environmet variables
        function obj = setupEnvironments(obj, type, obs_pos)
            obj.static_goal = [-1.43;0];
            if type == 1
                obj.n_obstacles = 0;
                obj.obstacle_matrix = [];
                obj.initial_zone = [-1.43;0;0.5]; %x,y,width
                obj.end_zone = [1.42;0;0.5]; %x,y,width
                obj.ref_waypoints = [1.42;0];

            elseif type == 2
                obj.n_obstacles = 1;
                obj.obstacle_matrix = [0; %, -0.40, 0.30;    % x
                    0.0; % 0.70, -0.8;   %y
                    0.25]; %0.2,0.2];     %r
                obj.initial_zone = [-1.43;0;0.5]; %x,y,width
                obj.end_zone = [1.42;0;0.5]; %x,y,width
                obj.ref_waypoints = [1.42;0];

            elseif type == 3
                if ~isempty(obs_pos)
                    obj.n_obstacles = size(obs_pos,2);
                    obj.obstacle_matrix = obs_pos;
                else
                    obj.n_obstacles = 0;
                    obj.obstacle_matrix = [];
                end


            else
                error("Environment not available");
            end

        end

        function [figure_handle, ax] = showEnvironment(obj)
            ll = obj.workspace_dimension.length/2;
            ur = obj.workspace_dimension.width/2;
            figure_handle = figure;
            ax = axes('Parent',figure_handle);
            rectangle(ax, 'Position',[-ll -ur obj.workspace_dimension.length obj.workspace_dimension.width],'EdgeColor','k',...
                'LineWidth',3)
            hold on
            % plot en zone and goal zone
            if obj.environment_type == 1 || obj.environment_type == 2
                rectangle(ax,'Position',[-1.7, -1.2, 0.5,2.4],'Curvature',1,'EdgeColor','b')
                hold on
                rectangle(ax,'Position',[1.36, -1.2, 0.5,2.4],'Curvature',1,'EdgeColor','g')
    
            else
                % plot waypoiny here
            end

            % pot obstacles
            if obj.n_obstacles ~= 0
                for i = 1:obj.n_obstacles
                    hold on
                    viscircles(ax,obj.obstacle_matrix(1:2,i)', obj.obstacle_matrix(3,i))

                end
            end
            xlim(ax, [-ll +ll])
            ylim(ax, [-ur +ur])
        end

       


    end

    methods (Static)
         function plotLive(ax_handle, q)
            s =size(q,2);
            col = ["r.","g.","k.","b.","c."];
            hold(ax_handle,"on")
            for i = 1:s
                plot(ax_handle,q(1,i), q(2,i),col(i),"MarkerSize",8 );
            end

        end

    end



end