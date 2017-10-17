%%
clear all;
clc;
close all;

%%
dir = strcat(pwd, '/../../data/results/absolute_coordinates/');
kinematics = '_Kinematics_q.sto';
body = '_BodyKinematics_pos_global.sto';
force = '_TaskForces.sto';
analytics = '_TaskAnalytics.sto';
station = '_StationPointAnalysis.sto';
id = '_ID.sto';
ra = '_ReactionAnalysis_ReactionLoads.sto';
constraints = '_ConstraintAnalysis.sto';
prefix1 = 'CONS_DS';
prefix2 = 'CONS_AG';
prefix3 = 'IMPL';

lineType1 = 'b--';
lineType2 = 'r:';
lineType3 = 'g.';

kinFile1 = strcat(dir, prefix1, kinematics);
kinFile2 = strcat(dir, prefix2, kinematics);
bodFile1 = strcat(dir, prefix1, body);
bodFile2 = strcat(dir, prefix2, body);

motTor1 = strcat(dir, prefix1, force);
motTor2 = strcat(dir, prefix2, force);
motTor3 = strcat(dir, prefix3, id);

motCon1 = strcat(dir, prefix1, analytics);
motCon2 = strcat(dir, prefix2, analytics);
motCon3 = strcat(dir, prefix3, ra);

const1 = strcat(dir, prefix1, constraints);
const2 = strcat(dir, prefix2, constraints);

%% coordinates

nRows = 3;
figure;
plotCoordinates(bodFile1, nRows, lineType1);
plotCoordinates(bodFile2, nRows, lineType2);
%suptitle('Body Kinematics');
hold off;

%% motion forces

nRows = 2;
figure;
plotTaskForces(motTor1, nRows, lineType1);
plotTaskForces(motTor2, nRows, lineType2);

idAnalysis = readMotionFile(motTor3);
idData = idAnalysis.data;
subplot(nRows, 6, 3);
hold on;
plot(idData(2:end, 1), idData(2:end, 2), lineType3);
xlim =[idData(1, 1), idData(end, 1)];
subplot(nRows, 6, 9);
hold on;
plot(idData(2:end, 1), idData(2:end, 3), lineType3);
xlim =[idData(1, 1), idData(end, 1)];
legend('Model-A', 'Model-B', 'ID', 'Location','south')
%suptitle('Motion Forces');
hold off;

%% reaction forces
nRows = 2;
figure;
plotAnalytics(motCon2, nRows, lineType1, 'reaction_forces');
plotAnalytics(motCon1, nRows, lineType2, 'reaction_forces');
%plotAnalytics(motCon1, nRows, lineType3, 'lambda');

raAnalysis = readMotionFile(motCon3);
raData = raAnalysis.data;
sign =   [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1];
%sign = [-1, -1, -1, -1, -1, -1];
index =  [5, 6, 7, 2, 3, 4, 14, 15, 16, 11, 12, 13];
%index =  [2, 3, 4, 11, 12, 13];
coords = [1:12];
for i = 1:length(index)
    subplot(nRows, 6, coords(i));
    hold on;
    plot(raData(:, 1), sign(i) * (raData(:, index(i))), lineType3);   
end
legend('Model-A', 'Model-B', 'RA')
%suptitle('Reaction Forces');
hold off;

% %% lambda 
% nRows = 2;
% figure;
% plotAnalytics(motCon1, nRows, lineType1, 'lambda');
% plotAnalytics(motCon2, nRows, lineType2, 'lambda');
% suptitle('Estimated Lagrange Multipliers');
% 
% %% const 
% nRows = 2;
% figure;
% plotAnalytics(const1, nRows, lineType1, 'state');
% plotAnalytics(const2, nRows, lineType2, 'state');
% suptitle('Actual Lagrange Multipliers');
% 
% 
% %% const 
% nRows = 2;
% figure;
% plotAnalytics(motCon1, nRows, lineType1, 'constraint');
% plotAnalytics(motCon2, nRows, lineType2, 'constraint');
% suptitle('Estimated Constraint Forces');
% 
% %% const 
% nRows = 2;
% figure;
% plotAnalytics(motCon1, nRows, lineType1, 'magnitude');
% plotAnalytics(motCon2, nRows, lineType2, 'magnitude');
% suptitle('Estimated Magnitude');
% 
% %% const 
% nRows = 2;
% figure;
% plotAnalytics(motCon1, nRows, lineType1, 'task');
% plotAnalytics(motCon2, nRows, lineType2, 'task');
% suptitle('Estimated Task Forces');
% 
