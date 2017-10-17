%%
clear all;
clc;
close all;

%%
dir = strcat(pwd, '/../../data/results/closed_kinematic_chain/');
kinematics = '_Kinematics_q.sto';
body = '_BodyKinematics_pos_global.sto';
force = '_TaskForces.sto';
analytics = '_TaskAnalytics.sto';
station = '_StationPointAnalysis.sto';
prefix1 = 'CONS_DS';
prefix2 = 'CONS_AG';
%prefix3 = 'UNCONSTRAINT';

nRows = 4;
lineType1 = 'b--';
lineType2 = 'r:';
%lineType3 = 'g.';

kinFile1 = strcat(dir, prefix1, kinematics);
kinFile2 = strcat(dir, prefix2, kinematics);
bodFile1 = strcat(dir, prefix1, body);
bodFile2 = strcat(dir, prefix2, body);
%kinFile3 = strcat(dir, prefix3, kinematics);
forceFile1 = strcat(dir, prefix1, force);
forceFile2 = strcat(dir, prefix2, force);
%forceFile3 = strcat(dir, prefix3, force);
analyFile1 = strcat(dir, prefix1, analytics);
analyFile2 = strcat(dir, prefix2, analytics);
%analyFile3 = strcat(dir, prefix3, analytics);
statiFile1 = strcat(dir, prefix1, station);
statiFile2 = strcat(dir, prefix2, station);

%%

figure;
plotCoordinates(kinFile1, nRows, lineType1);
plotCoordinates(kinFile2, nRows, lineType2);
%plotCoordinates(kinFile3, nRows, lineType3);
hold off;

figure;
plotCoordinates(bodFile1, nRows, lineType1);
plotCoordinates(bodFile2, nRows, lineType2);
%plotCoordinates(kinFile3, nRows, lineType3);
hold off;

figure;
plotTaskForces(forceFile1, nRows, lineType1);
plotTaskForces(forceFile2, nRows, lineType2);
%plotTaskForces(forceFile3, nRows, lineType3);
hold off;

figure;
plotAnalytics(analyFile1, nRows, lineType1, 'task');
plotAnalytics(analyFile2, nRows, lineType2, 'task');
%plotAnalytics(analyFile3, nRows, lineType3, 'task');
hold off;

figure;
plotAnalytics(analyFile1, nRows, lineType1, 'constraint');
plotAnalytics(analyFile2, nRows, lineType2, 'constraint');
%plotAnalytics(analyFile3, nRows, lineType3, 'constraint');
hold off;

figure;
plotAnalytics(analyFile1, nRows, lineType1, 'residual');
plotAnalytics(analyFile2, nRows, lineType2, 'residual');
%plotAnalytics(analyFile3, nRows, lineType3, 'residual');
hold off;

figure;
plotAnalytics(analyFile1, 2, lineType1, 'magnitude');
plotAnalytics(analyFile2, 2, lineType2, 'magnitude');
%plotAnalytics(analyFile3, 2, lineType3, 'magnitude');
hold off;

figure;
plotStationPointAnalysis(statiFile1, 2, lineType1, 'position');
plotStationPointAnalysis(statiFile2, 2, lineType2, 'position');
%plotStationPointAnalysis(statiFile3, 2, lineType3, 'rms');
hold off;