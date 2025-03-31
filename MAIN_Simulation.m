clc; clear all
format long
FigureSettings
close all;
load("colorblind_colormap.mat")

%% Load simulation settings
Settings_Simulation

% Solver tolerances
control.epsilon = 2e-2;
RelTol = 1e-4;
MaxStep = 1e-4;


% %% attack sequence
% simulation.attack.vehAtt = 1;
% simulation.att_5a_start1 = 0.5; simulation.att_5a_end1 = 1;
% simulation.att_5a_start2 = 2.5; simulation.att_5a_end2 = 3.0;
% simulation.att_5b_start = 2.0;  simulation.att_5b_end = 2.5;
% simulation.att_6a_start = 1.0;  simulation.att_6a_end = 1.5;
% simulation.att_6b_start = 1.5;  simulation.att_6b_end = 2.0;



%% Run Simulations
runScenario = 2;

if runScenario == 1         %% Only run steady state
    %% attack sequence
    simulation.attack.vehAtt = 1;
    simulation.att_5a_start1 = 0.5; simulation.att_5a_end1 = 2.5;
    simulation.att_5a_start2 = 8.5; simulation.att_5a_end2 = 10.5;
    simulation.att_5b_start = 6.5;  simulation.att_5b_end = 8.5;
    simulation.att_6a_start = 2.5;  simulation.att_6a_end = 4.5;
    simulation.att_6b_start = 4.5;  simulation.att_6b_end = 6.5;

    sim_T       = 10;
    sim_SteadyState

elseif runScenario == 2     %% Only run transient
    
    %% attack sequence
    simulation.attack.vehAtt = 1;
    simulation.att_5a_start1 = 0.5; simulation.att_5a_end1 = 2.5;
    simulation.att_5a_start2 = 8.5; simulation.att_5a_end2 = 10.5;
    simulation.att_5b_start = 6.5;  simulation.att_5b_end = 8.5;
    simulation.att_6a_start = 2.5;  simulation.att_6a_end = 4.5;
    simulation.att_6b_start = 4.5;  simulation.att_6b_end = 6.5;

    sim_T       = 10;
    sim_Transient

elseif runScenario == 3     %% Run both steady state and transient
    sim_SteadyState
    sim_Transient
else
end

