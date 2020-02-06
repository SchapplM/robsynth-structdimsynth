% Einstellungen für komb. Struktur- und Maßsynthese für Optimierung nach der
% Steifigkeit

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-01
% (C) Institut für Mechatronische Systeme, Universität Hannover

clc
clear

% Aufgaben-FG
DoF = [1 1 1 1 1 1];
Traj_no = 1;

Set = cds_settings_defaults(struct('DoF', DoF));
Set.task.Ts = 1e-2;
Set.task.Tv = 1e-1;
Set.task.profile = 0;
Set.task.maxangle = 10*pi/180; 
Traj = cds_gen_traj(DoF, Traj_no, Set.task);

Set.optimization.objective = 'stiffness';%'condition';
Set.optimization.optname = '3T3R_stiffness';
Set.optimization.constraint_obj(1) = 100; % Max. Masse 100kg
Set.optimization.NumIndividuals = 50;
Set.optimization.MaxIter = 20;
Set.general.plot_details_in_fitness = 1e3*0;
Set.general.plot_robot_in_fitness = 1e3;
Set.optimization.base_size = true;
Set.optimization.base_size_limits = [1 2];
Set.optimization.platform_size = true;
Set.optimization.platform_size_limits = [0.3 1];
Set.optimization.base_morphology = true;
Set.optimization.platform_morphology = true;
Set.optimization.movebase = false;
Set.optimization.use_desopt = true;
Set.general.max_retry_bestfitness_reconstruction = 1;
Set.general.verbosity = 3;
Set.general.matfile_verbosity = 0;
Set.optimization.ee_rotation = false;
% Nur Auswahl Kugelgelenk-Ende-PKM

Set.structures.whitelist = {'P6PRRRRR6V2G8P4A1'}; % '',  S6RRPRRR14
% Set.structures.whitelist = { ...
%   'P6PRRRRR68P1A1', 'P6PRRRRR6V2G8P1A1', ...
%   'P6PRRRRR6G8P4A1', 'P6PRRRRR6V2G8P4A1', ...
%   'P6RRRRRR10G1P1A1', 'P6RRRRRR10V3G1P1A1'};

cds_start
