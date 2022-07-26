function [r,T] = import_data(file_path)
%     arguments
%         file_path {mustBeText}
%     end 
    disp("Plotting file: " + file_path)
    if exist(file_path)
        disp("File exist")
        r = true;
        T = readtable(file_path);
        
    else
        warningMessage = sprintf('%s does not exit', file_path)
        r = false;
    end

end