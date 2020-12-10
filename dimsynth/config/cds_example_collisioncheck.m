% Einstellungen für komb. Struktur und Maßsynthese mit Kollisionsprüfung
% Ergebnis:
% Während der Optimierung werden Bilder zur Plausibilisierung der
% Nebenbedingungen gezeichnet

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-05
% (C) Institut für Mechatronische Systeme, Universität Hannover

clc
clear

% Funktion für Kollisionsprüfung kompilieren (falls noch nicht getan):
% matlabfcn2mex({'check_collisionset_simplegeom'})

% Aufgaben-FG
DoF = [1 1 1 1 1 1];
Traj_no = 1;

Set = cds_settings_defaults(struct('DoF', DoF));
Set.task.Ts = 1e-2;
Set.task.Tv = 1e-1;

Traj = cds_gen_traj(DoF, Traj_no, Set.task);

Set.optimization.objective = 'mass';
Set.optimization.optname = 'collisioncheck_test';
Set.optimization.NumIndividuals = 50;
Set.optimization.MaxIter = 10;
Set.optimization.ee_rotation = false;
Set.general.plot_details_in_fitness = 1e4*4e5; % Siehe cds_constraints.m; Debug-Plots für Selbstkollision
Set.general.plot_robot_in_fitness = 1e3;
Set.general.max_retry_bestfitness_reconstruction = 1;
Set.general.verbosity = 3;
Set.general.matfile_verbosity = 3;
Set.general.debug_calc = true;
Set.general.noprogressfigure = true;
Set.optimization.constraint_collisions = true;
Set.structures.whitelist = {'S6RRRRRR10', 'P6RRRRRR10G1P4A2'};
% Begrenze die Euler-Winkel, damit es mehr funktionierende Lösungen gibt
Set.task.maxangle = 10*pi/180;
   
cds_start
