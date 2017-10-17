%%
clear all;
clc;
close all;

%%
dir = strcat(pwd, '/../../data/results/taskspace/');
file = 'Controls_Controls.sto';

figType = '-dpng';
numOfRows1 = 6;
numOfRows2 = 3;
h = 3048;
w = 5096;
print = 0;
prefix = 'reaching-';

%% plot activations
if exist(strcat(dir, file)) 
    
    % get data
    controls = readMotionFile(strcat(dir, file));

    [r n] = size(controls.data);
    data = controls.data;
    labels = controls.labels;
    
    index = 1:r;
    time = data(:, 1);
    baseLine = 0;
   
    reserve = findStr(labels, 'reserve');
    nr = n - reserve(1);
    na = n - nr - 1;
    
    ma = ceil(na / numOfRows1);
    controls = figure;
    j = 1;
    for i = 2:na-1
        ax = subplot(numOfRows1, ma, j);
        plot(data(:, 1), smooth(data(:, i)));
        hold on;
        d = data(:, i);
        h1 = fill(time(index([1 1:end end])),... 
              [baseLine d(index)' baseLine],...
              'r', 'EdgeColor', 'none', 'facealpha',.2);
        title(labels(i), 'Interpreter', 'none');
        xlabel('time (s)');
        ylabel('controls');
        
        ax.XLim =[data(1, 1), data(end, 1)];
        ax.YLim = [0, 1];   
        j = j + 1;
    end
    
    if print == 1
        saveFigure(controls, strcat(dir, prefix, 'controls'), figType, 1);
    end
    
    ma = ceil(nr / numOfRows2);
    residuals = figure;
    j = 1;
    for i = reserve(1):reserve(end)
        ax = subplot(numOfRows2, ma, j);
        plot(data(:, 1), smooth(data(:, i)));
        hold on;
        d = data(:, i);
        h1 = fill(time(index([1 1:end end])),... 
              [baseLine d(index)' baseLine],...
              'r', 'EdgeColor', 'none', 'facealpha',.2);
        title(labels(i), 'Interpreter', 'none');
        xlabel('time (s)');
        ylabel('controls');
        
        ax.XLim =[data(1, 1), data(end, 1)];
        ax.YLim = [-1, 1];  
        j = j + 1;
    end
    
    if print == 1
        saveFigure(residuals, strcat(dir, prefix, 'residuals'), figType, 1);
    end
end

