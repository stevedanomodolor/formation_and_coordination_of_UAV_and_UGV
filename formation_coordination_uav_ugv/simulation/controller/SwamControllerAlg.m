classdef SwamControllerAlg 
    properties
        formation_param % information on paramters
        gain_param % struct gains c1 c2 for alpha(agent) c1 c2(obstacle) c1 c2(gamma)
        indx %position of the robot in the distance matrix
        integral_error % store the integrator
        previous_error % store for integral control


    end

    properties (Access = private)
        robot_size  %impose robot safety circle diameter
        id % to indxentify the robot
    end

    properties (Constant)
        safe_distance = 0.15 % safety distance around the robot in
    end


    methods
        % -- constructor
        function thisSwamControllerAlg = SwamControllerAlg(robot_size, id,indx,form_param, gains)
            % Important, this computation is valindx for 2d positions, 3d not
            % included
            thisSwamControllerAlg.robot_size = robot_size+thisSwamControllerAlg.safe_distance;
            thisSwamControllerAlg.indx = indx;
            thisSwamControllerAlg.id = id;
            thisSwamControllerAlg.gain_param  =  gains;
            thisSwamControllerAlg.integral_error = [0;0];
            thisSwamControllerAlg.previous_error = [0;0];

            % recompute c2 gain to ensure the following conditions
            %c1_apha < c1_gamma< c1_beta
            if false %gains.c1_alpha > gains.c1_gamma || gains.c1_gamma > gains.c1_beta
                error("Gain conditions not meet: c1_alpha < c1_gamma < c1_beta");
            end
            % recompute c2  gain based on the following equation c2 =
            % c1_gamma = s*sqrt(c) based on original paper
            thisSwamControllerAlg.formation_param = form_param;
            % ensure 0 < a < b is met
            if (0 > form_param.a) && (form_param.a >form_param.b)
                error('Error. 0 < a <= b not met');

            end
            % safety
            if form_param.nav_type <= 0 || form_param.nav_type >2
                error("Navigational_type should be 1-converge approach or 2 - parallel approach");
            end
            if form_param.nav_type ~= 2
                thisSwamControllerAlg.gain_param.c2_alpha = 2*sqrt(thisSwamControllerAlg.gain_param.c1_alpha);
                thisSwamControllerAlg.gain_param.c2_beta = 2*sqrt(thisSwamControllerAlg.gain_param.c1_beta);
                thisSwamControllerAlg.gain_param.c2_gamma = 2*sqrt(thisSwamControllerAlg.gain_param.c1_gamma); 
            end
            % formation
            thisSwamControllerAlg.formation_param.r = form_param.k*form_param.dist; % Neighbourhood distance


            % obstacle avoindxance parameters
            thisSwamControllerAlg.formation_param.d_obs = form_param.ratio*form_param.dist;
            thisSwamControllerAlg.formation_param.r_obs = form_param.k*thisSwamControllerAlg.formation_param.d_obs;
            if thisSwamControllerAlg.formation_param.d_obs < thisSwamControllerAlg.robot_size/2
                error("Minimum distance to obstacle is too low and can be dangerous, increase d/d_ob ratio");
            end


            % perform some formation configuration;
            thisSwamControllerAlg = thisSwamControllerAlg.configureFormation();
            disp("Robot: "  + id + " Initialized");


        end
        % -- Function responsible forcreating the formation parameters
        function thisSwamControllerAlg = configureFormation(thisSwamControllerAlg)
            % ensure distance is more thatn robot safe area
            dist = thisSwamControllerAlg.formation_param.dist;
            form_t = thisSwamControllerAlg.formation_param.type;
            if dist < (thisSwamControllerAlg.robot_size/2)
                error("Distance between robot will result in collision minimum radius distance: " + thisSwamControllerAlg.robot_size/2);
            end

            % formation configuration
            if form_t == 2
                % 2 man formation following
                thisSwamControllerAlg.formation_param.iad = [0,dist;    %interagent distance
                    dist,0];
                thisSwamControllerAlg.formation_param.N = 2; % number of agent
                thisSwamControllerAlg.formation_param.node_names = {'d1','d2'};



            elseif form_t == 3 % triangular formation
                thisSwamControllerAlg.formation_param.iad = [0,dist, dist;    %interagent distance
                    dist, 0, dist;
                    dist, dist, 0];
                thisSwamControllerAlg.formation_param.N = 3; % number of agent
                thisSwamControllerAlg.formation_param.node_names = {'d1','d2','d3'};

            elseif form_t == 4 % square formation
                h = sqrt(2*dist*dist); % diagonal distance
                thisSwamControllerAlg.formation_param.iad = [0,dist, h, dist;    %interagent distance
                    dist, 0,dist, h;
                    h,dist,0,dist;
                    dist,h, dist,0];
                thisSwamControllerAlg.formation_param.N = 4; % number of agent
                thisSwamControllerAlg.formation_param.node_names = {'d1','d2','d3','d4'};


            elseif form_t == 5 % pentagon formation
                h = dist + 2*dist*cos(deg2rad(72)); %diagonal distance
                thisSwamControllerAlg.formation_param.iad = [0,dist,h,h,dist;    %interagent distance
                    dist, 0, dist,h,h;
                    h,dist,0,dist, h;
                    h,h,dist,0,dist;
                    dist,h,h,dist,0];
                thisSwamControllerAlg.formation_param.N = 5; % number of agent
                thisSwamControllerAlg.formation_param.node_names = {'d1','d2','d3','d4','d5'};

            else
                error("Wrong formation type: triangle:1 square:2 pentagon: 3");
            end
            disp("Interagent distance matrix: ");
            disp(thisSwamControllerAlg.formation_param.iad);
        end

        % -- display formation
        function showFormation(thisSwamControllerAlg)
            figure
            if thisSwamControllerAlg.formation_param.type > 2
            pgon1 = nsindxedpoly(thisSwamControllerAlg.formation_param.N,'Center',[0 0],'SindxeLength',thisSwamControllerAlg.formation_param.dist);
            plot(pgon1)
            for i =1:thisSwamControllerAlg.formation_param.N
                text(pgon1.Vertices(i,1),pgon1.Vertices(i,2),"d" + string(i));

            end
            elseif  thisSwamControllerAlg.formation_param.type == 2
                plot([0; thisSwamControllerAlg.formation_param.dist], [0,0])
                text(0,0,"d1")
                text(thisSwamControllerAlg.formation_param.dist,0,"d2")

            end
%             axis equal

        end

        % -- main controller
        function [input, input_vec,q_obs_vector] = controller(thisSwamControllerAlg,q,p,ref,ob, ori, cooperation)
            % q is the current position of each of the drone- column vector
            % p is the current velocitu of each of the drones
            % ob is the current position of the obstacles [x,y,r]
            % orientatoin in degs
            if size(q,1) > 2
                error("Controller is intended for 2d plane, not 3d, send x y inputs");
            end

            % ----- formation computation
            q_rf = q'; % convert to row format
            [ind,dist] = knnsearch(q_rf,q_rf(thisSwamControllerAlg.indx,:),"K",thisSwamControllerAlg.formation_param.N);
            I = find(dist <= thisSwamControllerAlg.formation_param.r);
            if size(I,2) == 1 % only robot % dangerours because in real world the position can be different for the same robot, do not implement this in the real robot
                warning("Robot does not seem to have any neighbour, make sure this was done intentionally"); % TODO implement better logging
            end
            N_agent = ind(1,1:size(I,2));% agent neigbours
            s_Na = size(N_agent,2);
            u_formation = [0;0];
            grad_term = [0;0];
            cons_term = [0;0];
            deriv_term = [0;0];
            a = thisSwamControllerAlg.formation_param.a;
            b = thisSwamControllerAlg.formation_param.b;
            h_alpha = thisSwamControllerAlg.formation_param.h_alpha;
            eps = thisSwamControllerAlg.formation_param.eps;
            r = thisSwamControllerAlg.formation_param.r;
            qi =  q(:,thisSwamControllerAlg.indx);
            pi_ = p(:,thisSwamControllerAlg.indx);



            for i = 1: s_Na
                if N_agent(i) ~= thisSwamControllerAlg.indx % only for neigbours
                    qj = q(:,N_agent(i));
                    pj = p(:,N_agent(i));
                    if cooperation == 1
                    d = thisSwamControllerAlg.formation_param.iad(thisSwamControllerAlg.indx,N_agent(i));
                    else
                    d = thisSwamControllerAlg.formation_param.follow_dist;
                    end
                    % gradient term
                    fi = thisSwamControllerAlg.computeFi(qj,qi,a,b,h_alpha,eps,r,d);
                    nij = SwamControllerAlg.computeNij(qj,qi, eps);
                    grad_term = grad_term + fi*nij;
                    % consesus term
                    if thisSwamControllerAlg.formation_param.integrator == 2
                        aij = SwamControllerAlg.computeAij(qj,qi,r,eps,h_alpha);
                        cons_term = cons_term +aij*(pj-pi_);

                        deriv_term = deriv_term + aij*(qj-qi);

                    end
                     
            
                end

            end
            error_grad_term = grad_term;
            u_formation = grad_term*thisSwamControllerAlg.gain_param.c1_alpha + cons_term*thisSwamControllerAlg.gain_param.c2_alpha;
            % obstacle formation
            % TODO current implementation only consindxers circular
            % Todo plot gradient and potencial
            % obstacles, include hyperplane

            if isempty(ob)
                u_obstacle = [0;0];
                q_obs_vector = [0;0];

            else
                % compute position and velocity of obstacles
                % projection of agent onto obstacle
                ob_s = size(ob,2);
                r_obs = thisSwamControllerAlg.formation_param.r_obs;
                d_obs = thisSwamControllerAlg.formation_param.d_obs;
                h_beta = thisSwamControllerAlg.formation_param.h_beta;

                %                 q_hat = zeros(2,ob_s); % positions of all the obstables
                %                 p_hat = zeros(2,ob_s); % velocities of all the obstacles
                %                 N_beta = [];
                q_obs_vector = zeros(2,ob_s);
                    grad_term_obs = [0;0];
                    cons_term_obs = [0;0];
                    nav_term_obs = [0;0];
                for o = 1:ob_s
                    yk = ob(1:2,o); % center of obstacle
                    Rk = ob(3,o); % radius of obstacle
                    mu = Rk/norm(qi-yk);
                    ak = (qi-yk)/norm(qi-yk);
                    a__ = ak*ak';
                    sa = size(a__,1);
                    P = eye(sa)- a__;
                    q_hat_i = mu*qi + (1-mu)*yk;
                    p_hat_i = mu *P*pi_;
                  
%                     qi = thisSwamControllerAlg.indx;
%    if thisSwamControllerAlg.indx == 1
%                     disp("Here");
%                 end
                    d2obs = norm(qi- q_hat_i);
                    q_obs_vector(:,o) = q_hat_i;
                    if d2obs < r_obs %Neighbourhood obstacles of agent
                        % gradient term
                        fi_beta = SwamControllerAlg.computeFiB(q_hat_i,qi,h_beta,d_obs,eps);
                        nij = SwamControllerAlg.computeNij(q_hat_i,qi, eps);
                        grad_term_obs = grad_term_obs + fi_beta*nij;
                        % consesus term
                        if thisSwamControllerAlg.formation_param.integrator == 2

                            bij = SwamControllerAlg.computeBij(q_hat_i,qi,d_obs,eps, h_beta);
                            cons_term_obs = cons_term_obs +bij*(p_hat_i-pi_);
                        end
% 
%                          nav_term_obs = nav_term_obs -thisSwamControllerAlg.gain_param.c1_gamma*(SwamControllerAlg.sigmaOne(q_hat_i-ref.q))- ...
%                         thisSwamControllerAlg.gain_param.c2_gamma*(p_hat_i-ref.p);
                    end
                end
%              
                u_obstacle = thisSwamControllerAlg.gain_param.c1_beta* grad_term_obs ...
                    +thisSwamControllerAlg.gain_param.c2_beta*cons_term_obs;

            end

            % navigational term
            % TODO: implement wait until formation potential has reach a
            % threshold
            u_navigation = [0;0];
            if cooperation == 1
                if thisSwamControllerAlg.formation_param.nav_type == 1
                    % convergence approach
                    u_navigation = -thisSwamControllerAlg.gain_param.c1_gamma*(SwamControllerAlg.sigmaOne(qi-ref.q))- ...
                        thisSwamControllerAlg.gain_param.c2_gamma*(pi_-ref.p);
                else
                    %parallel approach
                    % compute the centroindx info
                    mean_q = [mean(q(1,:)); mean(q(2,:))];
                    mean_p = [mean(p(1,:)); mean(p(2,:))];
    
                    u_navigation = -thisSwamControllerAlg.gain_param.c1_gamma*(SwamControllerAlg.sigmaOne((mean_q-ref.q)))- ...
                        thisSwamControllerAlg.gain_param.c2_gamma*(mean_p-ref.p);
%                     u_navigation = sat_func(u_navigation,0.05);
     
                
                end
            end


            % orientation term
            % compute the centroindx vector
            u_orientation = [0;0];
            if ~isempty(ori)
                    desired_angle = deg2rad(ori);
                    % convert to 3d
                    desired_vector = [cos(desired_angle); sin(desired_angle);0];
                    if cooperation == 1
                    q_centroid = [mean(q,2);0];
                    q_leader = [q(:,1);0];
                    else
                        q_centroid = [q(:,1);0];
                        q_leader = [q(:,2);0];

                    end
                    dlc = norm(q_leader-q_centroid); % distance from centroid to leader drone
                    lcv = q_leader-q_centroid; % vector from centroid to leader
                    dcv = dlc*desired_vector; % vector from centroid to desired final pose of leader
                    acl =atan2(lcv(2), lcv(1));%angle of vector from centroid to leadership dron 
                    acc = atan2(dcv(2), dcv(1)); % angle of vector from fron centroid to final drone pose
                    if cooperation == 2
                        acl =atan(lcv(2)/ lcv(1));%angle of vector from centroid to leadership dron 
                        acc = atan(dcv(2)/dcv(1)); % angle of vector from fron centroid to final drone pose
                    end
                    alpha = acc-acl;%angle between vector lcv and dcv
                    % ---- We want to compute the rotational acceleration
                    % direction
                    q_hat = [q(:,thisSwamControllerAlg.indx); 0];% current drone pose
                    cq_hat = q_hat - q_centroid;% vector from centroid to current drone
                    rot_d =cross(cq_hat, [0,0,-alpha]) ; %rotaiton direction
%                     disp(alpha*180/pi)
%                     pause(0.005)
                    if norm(rot_d) == 0
                        rot_d = [0;0];
                    else
                    rot_d = rot_d/norm(rot_d); % convert to unit vecor; 
                    end
                    u_orientation = thisSwamControllerAlg.gain_param.c1_theta*abs((alpha))*[rot_d(1);rot_d(2)];
           %% try sun phd controller controller 
           % we assume agent 1 and 2 are the orientation agents
%            if (thisSwamControllerAlg.indx == 1) || (thisSwamControllerAlg.indx == 2)
%                % compute the current vector from agent 1 to agent 2
%                % we normalize it 
%                 if thisSwamControllerAlg.indx == 1
%                     q1_o = q(:,1);
%                     q2_o = q(:,2);
%                     ang = 1;
%                 else 
%                     q2_o = q(:,1);
%                     q1_o = q(:,2);
%                     ang = -1;
%                 end
%                 % relative position
%                  A12_ = q2_o - q1_o;
%                  % desired relatic position angle is assume to be from q1
%                  % to q2                 
%                  dA12_ = ang *norm(A12_)* [cos(deg2rad(desired_angle));sin(deg2rad(desired_angle)) ];
% 
%                
%                 u_orientation = thisSwamControllerAlg.gain_param.c1_theta*( A12_ - dA12_);
% 
%            else
%                u_orientation = [0;0];
% 
%             
%            end
            end

            % integral controld
            % error obtained from the formation gradient term
            thisSwamControllerAlg.integral_error = thisSwamControllerAlg.integral_error + error_grad_term*thisSwamControllerAlg.formation_param.dt;
            % saturate error to prevent anti-winup
%             thisSwamControllerAlg.integral_error = sat_func( thisSwamControllerAlg.integral_error, thisSwamControllerAlg.formation_param.int_max);
            u_integral = thisSwamControllerAlg.gain_param.c1_delta * thisSwamControllerAlg.integral_error; % + deriv_term*thisSwamControllerAlg.gain_param.c2_delta;
%             u_derivative = ((error_grad_term-previous_error)/ thisSwamControllerAlg.formation_param.dt)*thisSwamControllerAlg.gain_param.c3_delta;
%             thisSwamControllerAlg.previous_error = error_grad_term;
%                 ce = error_grad_term;

            input = u_formation + u_obstacle + u_navigation+u_orientation + u_integral+ 0;
            input_vec = [u_formation,u_obstacle,u_navigation,u_orientation,u_integral]; % debug purposes
%             potential = [potential_formation,potential_obstacle, potential_navigation, potential_orietation];
        end

    end
    methods (Static)

        % - computes the sigma norm of a vector
        function out = computeSigmaNorm(z,eps)
            n = norm(z);
            out = (1/eps)*((sqrt(1+eps*n*n))-1);
        end
        % compute the fi alpha function
        function out = computeFi(qj,qi,a,b,h,eps,r,d)
            % qj is the neighbour
            % qi is the drone itself
            % sigma norms of the error-cut off radius- desired distance
            z_sn = SwamControllerAlg.computeSigmaNorm((qj-qi), eps); % this can be done outsindxe the function
            r_sn = SwamControllerAlg.computeSigmaNorm(r, eps);
            d_sn = SwamControllerAlg.computeSigmaNorm(d, eps);
            out = SwamControllerAlg.ph(z_sn/r_sn,h) * SwamControllerAlg.fi(a,b,(z_sn-d_sn));
        end


        % - computes the fi function
        function out = fi( a,b,z)
            c = abs(a-b)/sqrt(4*a*b);
            out = 0.5 * (((a+b)*SwamControllerAlg.sigmaOne(z+c)) + (a-b));
        end

        % - computes the ph bumper function
        function out = ph(z, h)
            % the bumper function maps R+ to [0,1]
            if max(size(z)) > 1
                error("Bumper function: value must be scalar")
            end
            if( z >= 0) && ( z<h)
                out = 1;
            elseif (z >= h) && (z<=1)
                out = 0.5*(1+cos(pi*(z-h)/(1-h)));
            else
                out = 0;
            end

        end

        % computes the sigma1
        function out = sigmaOne(z)
            n = norm(z);
            out = z/sqrt(1+(n*n));
        end

        % --- compute nij function neighbourhood function
        function out = computeNij(qj,qi, eps)
            % qj is the neighbour
            % qi is the drone itself
            z = qj -qi;
            n = norm(z);
            out = z/sqrt(1+(eps*n*n));


        end

        % -- compute the adjency matrix element
        function out = computeAij(qj,qi,r,eps,h)
            z_sn = SwamControllerAlg.computeSigmaNorm(qj-qi, eps);
            r_sn = SwamControllerAlg.computeSigmaNorm(r,eps);
            out = SwamControllerAlg.ph(z_sn/r_sn,h);
        end

        % -- helper function for obstacle
        % damping function
        function out = computeFiB(qj,qi,h_beta,d_obs,eps)
            z_sn = SwamControllerAlg.computeSigmaNorm(qj-qi,eps);
            d_obs_sn = SwamControllerAlg.computeSigmaNorm(d_obs,eps);
            out = SwamControllerAlg.ph(z_sn/d_obs_sn,h_beta) * (SwamControllerAlg.sigmaOne(z_sn-d_obs_sn)-1);
        end
        function out = computeBij(qj,qi,d_obs,eps, h)
            z_sn = SwamControllerAlg.computeSigmaNorm(qj-qi,eps);
            d_obs_sn = SwamControllerAlg.computeSigmaNorm(d_obs,eps);
            out = SwamControllerAlg.ph(z_sn/d_obs_sn,h);
        end
    end

end


