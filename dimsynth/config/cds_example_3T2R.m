% Einstellungen für komb. Struktur und Maßsynthese für 3T2R Mechanismen

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-09
% (C) Institut für Mechatronische Systeme, Universität Hannover

clc
clear

% Aufgaben-FG
DoF = [1 1 1 1 1 0];
Traj_no = 1;

Set = cds_settings_defaults(struct('DoF', DoF));
Set.task.Ts = 1e-2;
Set.task.Tv = 1e-1;

Traj = cds_gen_traj(DoF, Traj_no, Set.task);

Set.optimization.objective = 'energy';
Set.optimization.optname = '3T2R_test';
Set.optimization.NumIndividuals = 50;
Set.optimization.MaxIter = 10;
Set.general.plot_details_in_fitness = 1e3;
Set.general.plot_robot_in_fitness = 1e3;
Set.general.verbosity = 3;
Set.general.matfile_verbosity = 3;
Set.optimization.ee_rotation = false;
Set.optimization.ee_translation = false;
Set.structures.maxnumprismatic = 3; % für Portal-Systeme
Set.structures.use_parallel = false;
Set.structures.whitelist = {'S5PRRRR10'};
cds_start(Set,Traj);
