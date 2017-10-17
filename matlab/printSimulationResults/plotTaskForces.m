function [  ] = plotTaskForces(file, numOfRows, lineType)
%PLOTCOORDINATES Summary of this function goes here
%   Detailed explanation goes here

if exist(file) 
    
    force = readMotionFile(file);

    [r n] = size(force.data);
    data = force.data;
    labels = force.labels;
          
    m = ceil((n - 1) / numOfRows);
    j = 1;
    totalRMS = 0;
    for i = 2:n
        ax = subplot(numOfRows, m, j);
        hold on;
        small = find(abs(data(:, i)) < 1E-03);
        data(small, i) = 0;
        plot(data(:, 1), smooth(data(:, i)), lineType);
%         title(strcat(labels(i), ' RMS: ', num2str(rms(data(:, i)))), ...
%             'Interpreter', 'none');
        title(labels{i}, 'Interpreter', 'none');
        xlabel('time (s)');
        ylabel('forces (Nm | N)');
        
        ax.XLim = [0.01, data(end, 1)];
        %ax.YLim = [0, 1];   
        j = j + 1;
        totalRMS = totalRMS + rms(data(:, i));
    end
%     text(2, 0, ...
%         strcat('Applied Forces', ' total RMS: ', num2str(totalRMS)), ...
%         'HorizontalAlignment' ,'center', 'VerticalAlignment', 'top')
    totalRMS;
   
else
    disp('Cant find the file');
end

