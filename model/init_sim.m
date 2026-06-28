% init_sim.m
% Initializes the workspace and opens the 3D Cubli model

clc; clear; close all;

% 1. Load the parameters from the .mat file
load('model/test torque.mat');
load('model/test torque - Copy.mat');

% 2. Open the Simulink model
open_system('model/DoF3_Cubli_main.slx');

disp('Workspace loaded and model ready to run.');
