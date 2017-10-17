%%
clear all;
clc;
close all;

%%
dir = strcat(pwd, '/../../data/results/ikbatch/');
tsikPrefix = 'TSDIK5_';
ikPrefix = 'IK5_';
tsikMarkerErrorFile = strcat(tsikPrefix, 'MarkerError.sto');
ikMarkerErrorFile = strcat(ikPrefix, 'MarkerError.sto');
fgType = '-dpng';

% parameters for a gait cycle
lb = 0.55;
ub = 1.8;
ticks = [0:20:100];

%% plot marker error

if exist(strcat(dir, tsikMarkerErrorFile)) && exist(strcat(dir, ikMarkerErrorFile))
    tsikMarkerError = readMotionFile(strcat(dir, tsikMarkerErrorFile));
    ikMarkerError = readMotionFile(strcat(dir, ikMarkerErrorFile));
    
    [tsikR n] = size(tsikMarkerError.data);
    [ikR n] = size(ikMarkerError.data);
    tsikD = tsikMarkerError.data;
    ikD = ikMarkerError.data;
    l = tsikMarkerError.labels;

    % plot min/max RMS
    tI = findStr(l, 'time');
    tsikTime = tsikD(:, tI(1))';
    ikTime = ikD(:, tI(1))';
    maxI = findStr(l, 'max');
    tsikMaxErr = tsikD(:, maxI(1))';
    ikMaxErr = ikD(:, maxI(1))';
    minI = findStr(l, 'min');
    tsikMinErr = tsikD(:, minI(1))';
    ikMinErr = ikD(:, minI(1))';
    rmsI = findStr(l, 'RMS');
    tsikRms = tsikD(:, rmsI(1))';
    ikRms = ikD(:, rmsI(1))';
    baseLine = 0;
    tsikIndex = 1:tsikR;
    ikIndex = 1:ikR;

    markerError = figure;
    hold on;
        
    h1 = fill(tsikTime(tsikIndex([1 1:end end])),... 
              [baseLine tsikMaxErr(tsikIndex) baseLine],...
              'c', 'EdgeColor', 'k', 'facealpha',.4);
        
    h2 = fill(tsikTime(tsikIndex([1 1:end end])),...  
              [baseLine tsikMinErr(tsikIndex) baseLine],...
              'w','EdgeColor','k', 'facealpha',1.0);
          
    h3 = fill(ikTime(ikIndex([1 1:end end])),... 
              [baseLine ikMaxErr(ikIndex) baseLine],...
              'b', 'EdgeColor', 'k', 'LineStyle', '--', 'facealpha',.4);
        
    h4 = fill(ikTime(ikIndex([1 1:end end])),...  
              [baseLine ikMinErr(ikIndex) baseLine],...
              'w','EdgeColor','k', 'LineStyle', '--', 'facealpha',1.0);
          
    p1 = plot(tsikTime, tsikRms, 'r');
%     plot(tsikTime, tsikMaxErr, 'g');
%     plot(tsikTime, tsikMinErr, 'k');  
              
    p2 = plot(ikTime, ikRms, 'r--');
%     plot(ikTime, ikMaxErr, 'g--');
%     plot(ikTime, ikMinErr, 'k--');  
          
    ax = legend([p1, p2, h1, h3], 'TSDIK RMS', 'IK RMS', 'TSDIK min/max area', 'IK min/max area');

    limsy=get(gca,'YLim');
    xlim([lb, ub]);
    ind = find(ikTime > lb & ikTime < ub);
    indd = linspace(ind(1), ind(end), length(ticks));
    set(gca, 'XTick', ikTime(int16(indd)));
    set(gca, 'XTickLabel', ticks); % Change x-axis ticks labels.
    xlabel('gait cycle (%)');
    
    ylim([-0.001, 0.021]);
    title('Marker Error');
    %xlabel('time (s)');
    ylabel('error (m)');
    hold off;
    
    %saveFigure(markerError, strcat(dir, 'markerError'), fgType, 0);
    %save2pdf(strcat(dir, 'markerError'), markerError, 600);
end


