function [  ] = plotCoordinates(file, numOfRows, lineType)
%PLOTCOORDINATES Summary of this function goes here
%   Detailed explanation goes here

if exist(file) 
    
    motion = readMotionFile(file);

    [r n] = size(motion.data);
    data = motion.data;
    labels = motion.labels;
    
    m = ceil((n - 1) / numOfRows);
    j = 1;
    for i = 2:n
        ax = subplot(m, 6, j);
        hold on;
        plot(data(:, 1), smooth(data(:, i)), lineType);
        title(labels(i), 'Interpreter', 'none');
        xlabel('time (s)');
        ylabel('coordinate (deg | m)');
        
        ax.XLim =[data(1, 1), data(end, 1)];
        %ax.YLim = [0, 1];   
        j = j + 1;
    end
else
    disp('Cant find the file');
end


