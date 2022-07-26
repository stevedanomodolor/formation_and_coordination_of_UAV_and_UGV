function [time_offset, input_ofst,output_ofst ] = preprocess_data(time, input_, output_, plot_results)
    disp("Removing gain offset...");
    min_output = min(input_);
    input_ofst_gain = input_ - min_output;
    output_ofst_gain = output_ - min_output;
    disp("Removing time offset...");
%     offset_ind = find(input_ofst_gain > min(input_ofst_gain));
    input_ofst = input_ofst_gain;%(offset_ind:end);
    output_ofst = output_ofst_gain;%(offset_ind:end);
    time_offset = time(1:end) - time(1);

    if plot_results == true
         %% no preprocessing
        figure
        plot(time, input_)
        hold on
        plot(time, output_)
        hold off
        legend("Input", "Ouput")
        title("Input vs Output no processing")
        %% offset gain
        figure
        plot(time, input_ofst_gain)
        hold on
        plot(time, output_ofst_gain)
        hold off
        legend("Input", "Ouput")
        title("Input vs Output after removing gain offset")
        %% offset removed
        figure
        plot(time_offset, input_ofst)
        hold on
        plot(time_offset, output_ofst)
        hold off
        legend("Input", "Ouput")
        title("Input vs Output after removing gain + time offset")
    end


end