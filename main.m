%% 

% Dobot Control


clf
clear all
close all
clc
set(0,'DefaultFigureWindowStyle','docked');
figure('Name','dobot')


% Set Variables

view(2);
ws = [-0.5 0.5 -0.5 0.5 0 2];

dobot = Dobot(ws, '1');
dobot.model.teach

% dobot.calc_volume(10);
% axis equal
