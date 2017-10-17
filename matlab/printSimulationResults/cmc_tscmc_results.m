%% 
% compaters simulations results between CMC and TSCMC for a gait movement
% the comparison is made between the activations and residual forces

clear all;
clc;
close all;

%%
tscmcDir = strcat(pwd, '/../../data/results/taskspacecmc/');
tscmcConFile = 'TSCMC_Controls.sto';

cmcDir = strcat(pwd, '/../../data/dataset/gait/Gait2354_Simbody/ResultsCMC/');
cmcConFile = 'subject01_walk1_controls.sto';

figType = '-dpng';
numOfCols1 = 4;
numOfCols2 = 4;
siz = [ 29.7 42.0];
orientation = 'portrait';
print = 0;

% parameters for a gait cycle
lb = 0.55;
ub = 1.8;
ticks = [0:20:100];
        
%% plot activations
if exist(strcat(tscmcDir, tscmcConFile)) && exist(strcat(cmcDir, cmcConFile))
    
    % get data
    controlsTSCMC = readMotionFile(strcat(tscmcDir, tscmcConFile));
    controlsCMC = readMotionFile(strcat(cmcDir, cmcConFile));
    

    [tscmcR tscmcN] = size(controlsTSCMC.data);
    [cmcR cmcN] = size(controlsCMC.data);
    tscmcD = controlsTSCMC.data;
    labels = controlsTSCMC.labels;
    cmcD = controlsCMC.data;
    cmcIndex = 1:cmcR;
    tscmcIndex = 1:tscmcR;
    cmcT = cmcD(:, 1);
    tscmcT = tscmcD(:, 1);
    baseLine = 0;
   
    reserve = findStr(labels, 'reserve');
    nr = tscmcN - reserve(1);
    na = tscmcN - nr - 1;
    
    
    ma = ceil(na / numOfCols1);
    controls = figure;
    j = 1;
    for i = 2:na
        ax = subplot(ma, numOfCols1, j);
        plot(tscmcD(:, 1), smooth(tscmcD(:, i)), '--');
        hold on;
        cmc = cmcD(:, i);
        tscmc = tscmcD(:, i);
        if i < 49
            h1 = fill(cmcT(cmcIndex([1 1:end end])),... 
                [baseLine cmc(cmcIndex)' baseLine],...
                'r', 'EdgeColor', 'none', 'LineStyle', '-', 'facealpha',.2);
        else
            h1 = fill(tscmcT(tscmcIndex([1 1:end end])),... 
                [baseLine tscmc(tscmcIndex)' baseLine],...
                'r', 'EdgeColor', 'none', 'LineStyle', '-', 'facealpha',.2);
        end
        title(labels(i ), 'Interpreter', 'none');
        ylabel('controls');
        
        limsy=get(gca,'YLim');
        xlim([lb, ub]);
        ind = find(cmcT > lb & cmcT < ub);
        indd = linspace(ind(1), ind(end), length(ticks));
        set(gca, 'XTick', cmcT(int16(indd)));
        set(gca, 'XTickLabel', ticks); % Change x-axis ticks labels.
        xlabel('gait cycle (%)');
        
        %ax.XLim =[tscmcD(1, 1) + 0.05, tscmcD(end, 1) - 0.05];
        %xlabel('time (s)');
        ax.YLim = [0, 1];   
        j = j + 1;
    end
    
    if print == 1
        saveFigure(controls, strcat(tscmcDir, 'cmc-controls'), figType, ...
            siz, orientation);
    end
    
    ma = ceil(nr / numOfCols2);
    residuals = figure;
    j = 1;
    for i = reserve(1):reserve(end)
        ax = subplot(ma, numOfCols2, j);
        plot(tscmcD(:, 1), smooth(tscmcD(:, i)), '--');
        hold on;
        cmc = cmcD(:, i);
        plot(cmcT, cmc, 'r-');
%         h1 = fill(cmcT(cmcIndex([1 1:end end])),... 
%               [baseLine cmc(cmcIndex)' baseLine],...
%               'r', 'EdgeColor', 'none', 'facealpha',.2);
        lab = strrep(labels{i}, '_reserve', '');
        title(lab, 'Interpreter', 'none');
        ylabel('controls');
        
        limsy=get(gca,'YLim');
        xlim([lb, ub]);
        ind = find(cmcT > lb & cmcT < ub);
        indd = linspace(ind(1), ind(end), length(ticks));
        set(gca, 'XTick', cmcT(int16(indd)));
        set(gca, 'XTickLabel', ticks); % Change x-axis ticks labels.
        xlabel('gait cycle (%)');
        
        %xlabel('time (s)');
        %ax.XLim =[tscmcD(1, 1) + 0.05, tscmcD(end, 1) - 0.05];
        %ax.YLim = [0, 1];  
        j = j + 1;
    end
    if print == 1
        saveFigure(residuals, strcat(tscmcDir, 'cmc-residuals'), figType, ... 
            siz, orientation);
    end
end

