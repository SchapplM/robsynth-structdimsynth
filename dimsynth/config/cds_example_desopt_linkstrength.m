% Einstellungen für komb. Struktur und Maßsynthese für Optimierung der
% Segmentauslegung im Hinblick auf die Materialspannung (Dehngrenze)

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
Set.task.profile = 0; % Nur Eckpunkte / statischer Fall

Traj = cds_gen_traj(DoF, Traj_no, Set.task);

Set.optimization.objective = 'mass';
Set.optimization.optname = 'linkstrength_3T3R';
Set.optimization.NumIndividuals = 10;
Set.optimization.MaxIter = 5;
Set.optimization.ee_rotation = false;
Set.optimization.movebase = false;
Set.optimization.use_desopt = true;
Set.optimization.desopt_link_yieldstrength = true;
Set.general.plot_details_in_fitness = 0*1e3;
Set.general.plot_robot_in_fitness = 0*1e3;
Set.general.max_retry_bestfitness_reconstruction = 1;
Set.general.verbosity = 4;
Set.general.matfile_verbosity = 3;
Set.structures.whitelist = {'P6RRPRRR14V3G1P3A1', 'S6RRRRRR10V3'};

cds_start