function d_sys = obtain_discrete_model_fisrt_order(tc, g, plot_test_control, dt)
[A, B, C, D] = tf2ss(g,[tc,1]);
sys = ss(A,B,C,D);
d_sys = c2d(sys,dt);



if plot_test_control == true
    figure
    step(sys,'-',d_sys,'--')
    legend("Continuouse model", "Discrete model")
    %% dummy plot 
    N = 100;
    dt_v = 0:dt:10;
    s_m_ = size(dt_v,2);
    u = zeros(s_m_, 1);
    u(1:floor(s_m_/3)) = 0;
    
    u(floor(s_m_/3):floor(s_m_/3)*2) = 50;
    u(floor(s_m_/3)*2:end) = 100;

    y = zeros(s_m_,1);
    x = 0;
    for k = 1:(s_m_)
        y(k) = d_sys.C*x+d_sys.D*u(k);
        x = d_sys.A * x+ d_sys.B*u(k);

    end

    figure
    plot(dt_v, y)
    hold on 
    plot(dt_v,u)
    hold off
    title("Dummy test")
    legend("System response", "system input")

end



end