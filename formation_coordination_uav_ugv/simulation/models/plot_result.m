function [file_exit,T] = plot_result(T)
    % plot the velocities 
    figure
    %% VX
    subplot(8,1,1)
    plot(T.t, T.vx_i)
    ylabel("vx_l(mm/s)")
    xlabel("T(s)")
    %% Vy
    subplot(8,1,2)
    plot(T.t, T.vy_i)
    ylabel("vy_l(mm/s)")
    xlabel("T(s)")     
    %% Vz 
    subplot(8,1,3)
    plot(T.t, T.vz_i)
    ylabel("vz_l(mm/s)")
    xlabel("T(s)")  
    %% yaw
    subplot(8,1,4)
    plot(T.t, T.rateYaw)
    ylabel("yaw_rate_l(mrad/s)")
    xlabel("T(s)")  
    %% VX -c
    subplot(8,1,5)
    plot(T.t, T.vx_c_w)
    ylabel("vx_c(mm/s)")
    xlabel("T(s)")
    %% Vy -c
    subplot(8,1,6)
    plot(T.t, T.vy_c_w)
    ylabel("vy_c(mm/s)")
    xlabel("T(s)")     
    %% Vz -c
    subplot(8,1,7)
    plot(T.t, T.yaw_c_w)
    ylabel("vz_c(mm/s)")
    xlabel("T(s)")  
    %% yaw_ratecomand
    subplot(8,1,8)
    plot(T.t, T.zDistance)
    ylabel("yaw_rate_c(mrad/s)")
    xlabel("T(s)")  
    sgtitle('Velocity command- velocity value') 
    
    %% Plot positoins %%%%%%%%%%%%%%%%%%%%%%
    figure
    %% X
    subplot(8,1,1)
    plot(T.t, T.x_i)
    ylabel("x_l(mm/s)")
    xlabel("T(s)")
    %% y
    subplot(8,1,2)
    plot(T.t, T.y_i)
    ylabel("y_l(mm/s)")
    xlabel("T(s)")     
    %% z 
    subplot(8,1,3)
    plot(T.t, T.z_i)
    ylabel("z_l(mm/s)")
    xlabel("T(s)")  
    %% yaw
    subplot(8,1,4)
    plot(T.t, T.yaw_i)
    ylabel("yaw_l(mrad)")
    xlabel("T(s)")  
    %% VX -c
    subplot(8,1,5)
    plot(T.t, T.vx_c_w)
    ylabel("vx_c(mm/s)")
    xlabel("T(s)")
    %% Vy -c
    subplot(8,1,6)
    plot(T.t, T.vy_c_w)
    ylabel("vy_c(mm/s)")
    xlabel("T(s)")     
    %% Vz -c
    subplot(8,1,7)
    plot(T.t, T.yaw_c_w)
    ylabel("vz_c(mm/s)")
    xlabel("T(s)")  
    %% yaw_ratecomand
    subplot(8,1,8)
    plot(T.t, T.zDistance)
    ylabel("yaw_rate_c(mrad/s)")
    xlabel("T(s)")  
    sgtitle('Velocity command- position value') 
    %% plot acclerations %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    figure
    %% aX
    subplot(7,1,1)
    plot(T.t, T.ax)
    ylabel("vx_l(mm2/s)")
    xlabel("T(s)")
    %% ay
    subplot(7,1,2)
    plot(T.t, T.ay)
    ylabel("vy_l(mm2/s)")
    xlabel("T(s)")     
    %% az 
    subplot(7,1,3)
    plot(T.t, T.az)
    ylabel("vz_l(mm2/s)")
    xlabel("T(s)")  
    %% VX -c
    subplot(7,1,4)
    plot(T.t, T.vx_c_w)
    ylabel("vx_c(mm/s)")
    xlabel("T(s)")
    %% Vy -c
    subplot(7,1,5)
    plot(T.t, T.vy_c_w)
    ylabel("vy_c(mm/s)")
    xlabel("T(s)")     
    %% Vz -c
    subplot(7,1,6)
    plot(T.t, T.yaw_c_w)
    ylabel("vz_c(mm/s)")
    xlabel("T(s)")  
    %% yaw_ratecomand
    subplot(7,1,7)
    plot(T.t, T.zDistance)
    ylabel("yaw_rate_c(mrad/s)")
    xlabel("T(s)")  
    sgtitle('Velocity command- Accleration value') 

    
end