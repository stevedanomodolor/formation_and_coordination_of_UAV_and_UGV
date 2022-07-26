function [d_sys, std_error] = obtain_discrete_model_second_order(sys,plot_test_control, dt,t,  input, output,title_name )
[A, B, C, D] = tf2ss(sys.numerator, sys.denominator);
sys = ss(A,B,C,D);
d_sys = c2d(sys,dt);

  %% dummy plot 
     
    N = max(size(input)); % Size of input data 
    N = cast((N/2)+(N/4),"uint32");
    input = input(1:N,1);
    output = output(1:N,1);
    t = t(1:N,1);
    
    y = zeros(N,1);
    x = [0;0];
    for k = 1:(N)
        y(k) = d_sys.C*x;
        x = d_sys.A * x + d_sys.B * input(k);    
    end
 error = output-input;
    std_error = std(error);
if plot_test_control == true
    figure
    step(sys,'-',d_sys,'--')
    legend("Continuouse model", "Discrete model")
  

   
    figure
    plot(t, y)
    hold on 
    plot(t,input)
    hold on
    plot(t, output)
    hold off
    title(cast((join(["System indentification ", title_name])), "char"))
    xlabel("t(s)")
    ylabel("v(m/s)")
    legend("System response", "system input", "Experimental data")

    file_name  = ['models/', title_name ,'.csv'];
    table_final = [t,input,y,output];

    writematrix(table_final,file_name) 


end



end