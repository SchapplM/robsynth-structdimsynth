% Einstellungen für komb. Struktur- und Maßsynthese für 3T3R PKM

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-09
% (C) Institut für Mechatronische Systeme, Universität Hannover

clc
clear

% Aufgaben-FG
DoF = [1 1 1 1 1 1];
Traj_no = 1;

Set = cds_settings_defaults(struct('DoF', DoF));
Set.task.Ts = 1e-2;
Set.task.Tv = 1e-1;
Set.task.profile = 1;
Set.task.maxangle = 5*pi/180; 
Traj = cds_gen_traj(DoF, Traj_no, Set.task);

Set.optimization.objective = 'energy';
Set.optimization.optname = '3T3R_PKM';
Set.optimization.NumIndividuals = 20;
Set.optimization.MaxIter = 10;
Set.general.plot_details_in_fitness = 1e3;
Set.general.plot_robot_in_fitness = 1e3;
Set.optimization.base_size = false;
Set.optimization.platform_size = false;
% Set.optimization.movebase = false;
Set.optimization.base_morphology = true;
Set.optimization.platform_morphology = true;
Set.general.max_retry_bestfitness_reconstruction = 1;
Set.general.verbosity = 3;
Set.general.matfile_verbosity = 3;
Set.general.parcomp_struct = true; % Parallele Berechnung
Set.structures.use_serial = false;
Set.optimization.ee_rotation = false;

cds_start
