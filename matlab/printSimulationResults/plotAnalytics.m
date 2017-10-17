function [  euclid ] = plotAnalytics(file, numOfRows, lineType, filter)
%PLOTCOORDINATES Summary of this function goes here
%   Detailed explanation goes here

if exist(file) 
    
    analytics = readMotionFile(file);
    data = analytics.data;
    labels = analytics.labels;
    
    index = findStr(labels, filter);
    labels = {labels{1}, labels{index}};
    data = [data(:, 1), data(:, index)];
    [r n] = size(data);
 
    m = ceil((n - 1) / numOfRows);
    j = 1;
    for i = 2:n
        ax = subplot(numOfRows, m, j);
        hold on;
        small = find(abs(data(:, i)) < 1E-03);
        data(small, i) = 0;
        plot(data(:, 1), smooth(data(:, i)), lineType);
        title(labels{i}, 'Interpreter', 'none');
        xlabel('time (s)');
        
        if ~isempty(findstr(labels{i},'magnitude'))
            ylabel('magnitude');
        elseif ~isempty(findstr(labels{i},'kinetic_energy'))
            ylabel('KE (J)');
        else
            ylabel('forces (Nm | N)');
        end
        
        ax.XLim =[data(1, 1), data(end, 1)];
        %ax.YLim = [, 1];   
        %ax.XLim = [0, data(end, 1)];
        j = j + 1;
    end
else
    disp('Cant find the file');
end

