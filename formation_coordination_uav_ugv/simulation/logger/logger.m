classdef logger < handle

    % -- logger format
    % time position (x, y) velocities (x y) accelerations(x y)
    % command_position(x y) command_velocity(x,y) input_total(x y)
    % input_formation(x y) input_obstacle(x y) input_navigation(x y)

    properties
        n_robots
%         q % positions
%         p % velocites
%         a % accelerarions
%         cq % position command
%         cp % velocity command
%         int % total input
%         in_obs % input obstacle
%         in_for % input formation
%         in_nav % input navigation
        file_path
        file_name
        varnames % variables names
        tr % total number of rows
        log_table
        n_data
        robot_naming


    end
    methods
        % -- constructor
        function thisLogger = logger(filepath, filename, n_robots, n_data, robot_naming)
            thisLogger.file_path = filepath;
            thisLogger.file_name = filename; 
            thisLogger.n_robots = n_robots;
            thisLogger.n_data = n_data;
            thisLogger.varnames = cell(n_robots*n_data*2+5,1);
            thisLogger.tr = n_robots*n_data*2+5;
            thisLogger.robot_naming = robot_naming;
            thisLogger.varnames(1) = {'Time'};
            thisLogger.varnames(2) = {'cqx'};
            thisLogger.varnames(3) = {'cqy'};
            thisLogger.varnames(4) = {'cpx'};
            thisLogger.varnames(5) = {'cpy'};
            

            rp = 1;
            df = (n_data*2);

            for i = 6:df:thisLogger.tr
                thisLogger.varnames(i)  = strcat(thisLogger.robot_naming(rp),'_x');
                thisLogger.varnames(i+1)  =strcat(thisLogger.robot_naming(rp),'_y');
                thisLogger.varnames(i+2)  = strcat(thisLogger.robot_naming(rp),'_vx');
                thisLogger.varnames(i+3)  = strcat(thisLogger.robot_naming(rp),'_vy');
                thisLogger.varnames(i+4)  = strcat(thisLogger.robot_naming(rp),'_ax');
                thisLogger.varnames(i+5)  = strcat(thisLogger.robot_naming(rp), '_ay');
                thisLogger.varnames(i+6)  = strcat(thisLogger.robot_naming(rp), '_inx');
                thisLogger.varnames(i+7)  = strcat(thisLogger.robot_naming(rp), '_iny');
                thisLogger.varnames(i+8)  = strcat(thisLogger.robot_naming(rp), '_infx');
                thisLogger.varnames(i+9)  = strcat(thisLogger.robot_naming(rp), '_infy');
                thisLogger.varnames(i+10)  = strcat(thisLogger.robot_naming(rp), '_inox');
                thisLogger.varnames(i+11)  = strcat(thisLogger.robot_naming(rp),'_inoy');
                thisLogger.varnames(i+12)  = strcat(thisLogger.robot_naming(rp), '_innx');
                thisLogger.varnames(i+13)  = strcat(thisLogger.robot_naming(rp), '_inny');
                thisLogger.varnames(i+14)  = strcat(thisLogger.robot_naming(rp), '_inorx');
                thisLogger.varnames(i+15)  = strcat(thisLogger.robot_naming(rp), '_inory');
                thisLogger.varnames(i+16)  = strcat(thisLogger.robot_naming(rp), '_inintx');
                thisLogger.varnames(i+17)  = strcat(thisLogger.robot_naming(rp), '_ininty');
                rp = rp+1;

            end

        end

        function saveData(thisLogger)
              % convert array to table 
              thisLogger.varnames
              T = array2table(thisLogger.log_table,'VariableNames', thisLogger.varnames);
              % log data to file 
              file_loc = [thisLogger.file_path thisLogger.file_name  '.csv'];
              disp("Writting data to file: "+file_loc )
              writetable(T,file_loc);

        end
    
        function logData(thisLogger,t, cq,cp,q,p,a,inp,inpf,inpo,inpn,inpor,inpint)

            newLog = zeros(1, thisLogger.tr);
            newLog(1,1) = t;
            newLog(1,2) = cq(1);
            newLog(1,3) = cq(2);
            newLog(1,4) = cp(1);
            newLog(1,5) = cp(2);
            df = thisLogger.n_data*2;
            ind = 1;


            for i = 6:df:thisLogger.tr
                newLog(1,i) = q(1,ind);
                newLog(1,i+1) = q(2,ind);
                newLog(1,i+2) = p(1,ind);
                newLog(1,i+3) = p(2,ind);
                newLog(1,i+4) = a(1,ind);
                newLog(1,i+5) = a(2,ind);
                newLog(1,i+6) = inp(1,ind);
                newLog(1,i+7) = inp(2,ind);
                newLog(1,i+8) = inpf(1,ind);
                newLog(1,i+9) = inpf(2,ind);
                newLog(1,i+10) = inpo(1,ind);
                newLog(1,i+11) = inpo(2,ind);
                newLog(1,i+12) = inpn(1,ind);
                newLog(1,i+13) = inpn(2,ind);
                newLog(1,i+14) = inpor(1,ind);
                newLog(1,i+15) = inpor(2,ind);
                newLog(1,i+16) = inpint(1,ind);
                newLog(1,i+17) = inpint(2,ind);
                ind = ind+1;
            end
            

%             thisLogger.updateTable(newLog);
            thisLogger.log_table = [thisLogger.log_table; newLog];



        end
    

    end





end