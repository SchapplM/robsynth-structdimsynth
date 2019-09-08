% Einstellungen für komb. Struktur und Maßsynthese für 3T1R Mechanismen

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-09
% (C) Institut für Mechatronische Systeme, Universität Hannover

clc
clear

% Aufgaben-FG
DoF = [1 1 1 0 0 1];
Traj_no = 1;

Set = cds_settings_defaults(struct('DoF', DoF));
Set.task.Ts = 1e-2;
Set.task.Tv = 1e-1;

Traj = cds_gen_traj(DoF, Traj_no, Set.task);

Set.optimization.objective = 'mass';
Set.optimization.optname = '3T1R_test';
Set.optimization.NumIndividuals = 5;
Set.optimization.MaxIter = 5;
Set.general.plot_details_in_fitness = 1e3;
Set.general.plot_robot_in_fitness = 1e3;
Set.general.verbosity = 3;
Set.structures.maxnumprismatic = 3; % für Portal-Systeme

cds_start
