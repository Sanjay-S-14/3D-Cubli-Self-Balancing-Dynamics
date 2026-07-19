% init_sim.m
% Initializes the workspace and opens the 3D Cubli model

clc; clear; close all;

% 1. Load the required .mat file
load("model/physical_parameters");
load("model/tuning_parameter_corner");
load("model/tuning_parameter_edge");

% 2. Open the Simulink model
open_system('model/Non_Holonomical_Cubli.slx');

disp('Workspace loaded and model ready to run.');
