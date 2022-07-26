classdef droneSecondOrderModel < handle
    properties
        output
        state 
        N % number of state
        sx % vx system dynamics 
        sy % vy system dynamics
        id % keep track of the robot
        include_noise % include noise data 
        var_vx % noise variance vx 
        var_vy % noise variance vy
        double_integrator %use double integrator
        activate_stop = false
    end
    methods
        function obj = droneSecondOrderModel(q0, p0,a0, N, dsys, id, double_integrator,include_noise,var_vx, var_vy)
            
            if nargin == 7
                obj.state.p = p0;
                obj.state.q = q0;
                obj.state.a = a0;
                obj.state.vx =[p0(1); a0(1)] ;
                obj.state.vy = [p0(2); a0(2)] ;
                obj.N = N;
                obj.sx = dsys.vx;
                obj.sy = dsys.vy;
                obj.id = id;
                obj.double_integrator = double_integrator;
                obj.state.vf = p0;

            elseif nargin == 10
                obj.state.p = p0;
                obj.state.q = q0;
                obj.state.a = a0;
                obj.state.vx =[p0(1); a0(1)] ;
                obj.state.vy = [p0(2); a0(2)] ;
                obj.N = N;
                obj.sx = dsys.vx;
                obj.sy = dsys.vy;
                obj.id = id;
                obj.include_noise = include_noise;
                obj.var_vx = var_vx;
                obj.var_vy = var_vy;
                obj.double_integrator = double_integrator;
                obj.state.vf = p0;


            else
                error("Not enough argument")
            end
        end
        function display_state(obj)
            disp("position: " + obj.state.q);
            disp("Velocity: " + obj.state.p);

        end
        function obj = update_state(obj, input)
            % velocity as input in 2d [vx;vy]
            if max(size(input,1))  == obj.N
                 noise_vx = 0;
                 noise_vy = 0;
                 if obj.include_noise
                    noise_vx = normrnd(0,obj.var_vx);
                    noise_vy = normrnd(0, obj.var_vy);
                 end
                 if obj.activate_stop
                     obj.state.vf(1) = 0;
                     obj.state.vf(2) = 0;
                 elseif obj.double_integrator 
                    obj.state.a(1) = input(1);
                    obj.state.a(2) = input(2);
                    obj.state.vf(1) = obj.state.vf(1) + obj.sx.Ts*obj.state.a(1);
                    obj.state.vf(2) = obj.state.vf(2) + obj.sx.Ts*obj.state.a(2);
                 else 
                     obj.state.vf(1) = input(1);
                     obj.state.vf(2) = input(2);
                 end
              
                 obj.state.vx = obj.sx.A*obj.state.vx + obj.sx.B*obj.state.vf(1)+[noise_vx;0];
                 obj.state.vy = obj.sy.A*obj.state.vy + obj.sy.B*obj.state.vf(2) + [noise_vy;0];
                 obj.state.p(1) = obj.sx.C*obj.state.vx;
                 obj.state.p(2) = obj.sy.C*obj.state.vy;
                 % integrate to get position using euler integration 
                 obj.state.q(1) = obj.state.q(1) + obj.sx.Ts*obj.state.p(1);
                 obj.state.q(2) = obj.state.q(2) + obj.sx.Ts*obj.state.p(2);

                 % update output velocity position and acceleration
                 obj.output.a(1) = obj.state.a(1);
                 obj.output.a(2) = obj.state.a(2);
                 obj.output.p(1) = obj.state.p(1);
                 obj.output.p(2) = obj.state.p(2);
                 obj.output.q(1) = obj.state.q(1);
                 obj.output.q(2) = obj.state.q(2);
                 % update state 

            else
                error("Error. Incorrect input value size, it should be %d but is %d", obj.N, size(input,1));
            end

        end
        function output = get_state(obj)
                output = obj.output;
        end
        function stop_robot(obj)
              obj.activate_stop = true;
        end


        
    end
end