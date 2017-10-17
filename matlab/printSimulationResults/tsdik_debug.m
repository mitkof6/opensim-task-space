%%
clear all;
clc;
close all;

%%
dir = strcat(pwd, '/../../data/results/taskspaceik/');
%dir = strcat(pwd, '/../../data/results/taskspacemc/');
%dir = strcat(pwd, '/../../data/results/ikbatch/');
%prefix = 'TSIK5_';
%prefix = 'TOMC_';
prefix = 'TSDIK_';
stateFile = strcat(prefix, 'State.sto');
desiredCommandFile = strcat(prefix, 'DesiredCommands.sto');
markerErrorFile = strcat(prefix, 'MarkerError.sto');

%states = readMotionFile(strcat(dir, stateFile));

%% plot marker error

if exist(strcat(dir, markerErrorFile))
    markErr = readMotionFile(strcat(dir, markerErrorFile));
    
    [r n] = size(markErr.data);
    m = floor(sqrt(n - 1));
    if (m * m < n - 1)
       m = m + 1; 
    end
    d = markErr.data;
    l = markErr.labels;
    figure;
    for i = 1:n - 1 
        subplot(m, m, i);
        plot(d(:, 1), d(:, i + 1));
        title(l(i + 1), 'Interpreter', 'none');
        xlabel('time (s)');
        xlim([d(1, 1) d(end, 1)]);
    end
    
    % plot min/max
    tI = findStr(l, 'time');
    time = d(:, tI(1))';
    maxI = findStr(l, 'max');
    maxErr = d(:, maxI(1))';
    minI = findStr(l, 'min');
    minErr = d(:, minI(1))';
    rmsI = findStr(l, 'RMS');
    rms = d(:, rmsI(1))';
    baseLine = 0;
    index = 1:r;

    markError = figure;
    hold on;
    plot(time, rms, 'r--');
    plot(time, maxErr, 'g');
    plot(time, minErr, 'm');  
    h1 = fill(time(index([1 1:end end])),... 
              [baseLine maxErr(index) baseLine],...
              'b', 'EdgeColor', 'none', 'facealpha',.2);
        
    h2 = fill(time(index([1 1:end end])),...  
              [baseLine minErr(index) baseLine],...
              'w','EdgeColor','none', 'facealpha',1.0);
          
    legend('RMS', 'max', 'min', 'area');
    limsy=get(gca,'YLim');
    ylim([-0.001, limsy(2)]);
    title('Marker Error');
    xlabel('time (s)');
    ylabel('error');
    hold off;
       
end

%% plot desired command

if exist(strcat(dir, desiredCommandFile))
    desCom = readMotionFile(strcat(dir, desiredCommandFile));
    
    [r n] = size(desCom.data);
    m = floor(sqrt(n - 1));
    if (m * m < n - 1)
       m = m + 1; 
    end
    d = desCom.data;
    l = desCom.labels;
    figure;
    for i = 1:n - 1 
        subplot(m, m, i);
        plot(d(:, 1), d(:, i + 1));
        title(l(i + 1), 'Interpreter', 'none');
        xlabel('time (s)');
        xlim([d(1, 1) d(end, 1)]);
    end
end

