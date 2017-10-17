function [  ] = plotStationPointAnalysis(file, numOfRows, lineType, filter)
%PLOTCOORDINATES Summary of this function goes here
%   Detailed explanation goes here

if exist(file) 
    
    analysis = readMotionFile(file);
    data = analysis.data;
    labels = analysis.labels;
    
    index = findStr(labels, filter);
    labels = {labels{1}, labels{index}};
    data = [data(:, 1), data(:, index)];
    [r n] = size(data);
 
    m = ceil((n - 1) / numOfRows);
    j = 1;
    for i = 2:n
        ax = subplot(numOfRows, m, j);
        hold on;
        plot(data(:, 1), smooth(data(:, i)), lineType);
        title(labels{i}, 'Interpreter', 'none');
        xlabel('time (s)');
        
        if ~isempty(findstr(labels{i},'position'))
            ylabel('position (m)');
        elseif ~isempty(findstr(labels{i},'velocity'))
            ylabel('velocity (m/s)');
        else
            ylabel('acceleration (m/s^2)');
        end
        
        
        ax.XLim =[data(1, 1), data(end, 1)];
        %ax.YLim = [0, 0.5];   
        j = j + 1;
    end   
    subplot(numOfRows, m, j);
    hold on;
    plot(data(:, 3), data(:, 4), lineType);
    title('y-z', 'Interpreter', 'none');
    xlabel('y (m)');
    ylabel('z (m)');
else
    disp('Cant find the file');
end

