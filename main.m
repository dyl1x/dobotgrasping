%% 

% Dobot Control
close all
clearvars
clc
set(0,'DefaultFigureWindowStyle','docked');
figure('Name','dobot')


% Set Variables

view(2);
ws = [-0.5 0.5 -0.5 0.5 0 0.8];

dobot = Dobot(ws, '1');
dobot.model.teach

% dobot.calc_volume(10);
% axis equal
