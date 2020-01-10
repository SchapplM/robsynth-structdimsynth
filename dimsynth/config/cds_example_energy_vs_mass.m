% Einstellungen für komb. Struktur und Maßsynthese für Optimierung der
% dynamik-abhängigen Energie mit Nebenbedingung einer Untergrenze für die
% Masse. Dabei soll die Maßsynthese mit Nebenbedingung der Materialspannung
% verwendet werden.

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
Set.task.profile = 1; % Nur Eckpunkte / statischer Fall

Traj1 = cds_gen_traj(DoF, Traj_no, Set.task);
% Trajektorie reduzieren
Traj2 = rmfield(Traj1, 'XE');
Traj = timestruct_select(Traj2, [1,30]);
Traj.XE = Traj1.XE;
clear Traj1 Traj2
Set.general.debug_calc = false;
Set.optimization.objective = 'energy';
Set.optimization.constraint_obj(1) = 100; % max. 100kg
Set.optimization.constraint_obj(4) = 1e3; % Kondition darf nicht extrem schlecht sein
Set.optimization.optname = 'linkstrength_3T3R';
Set.optimization.NumIndividuals = 10;
Set.optimization.MaxIter = 5;
Set.optimization.ee_rotation = false;
Set.optimization.movebase = false;
Set.optimization.use_desopt = true;
Set.optimization.desopt_link_yieldstrength = true;
Set.optimization.max_range_active_revolute = 2*pi;
Set.general.plot_details_in_fitness = 0*1e3;
Set.general.plot_robot_in_fitness = 0*1e10;
Set.general.plot_details_in_desopt = 0*(-1e4);
Set.general.max_retry_bestfitness_reconstruction = 1;
Set.general.verbosity = 4;
Set.general.matfile_verbosity = 3;
Set.general.nosummary = true;
Set.structures.whitelist = {'P6RRPRRR14V3G1P3A1'}; % , S6RRPRRR14 'S6RRRRRR10V3'

cds_start

return
Set.general.regenerate_summmary_only = true;
Set.general.nosummary = false;
cds_start
