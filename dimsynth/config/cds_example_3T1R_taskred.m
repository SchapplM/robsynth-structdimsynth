% Einstellungen für komb. Struktur und Maßsynthese für 3T1R Mechanismen
% Aufgabenredundanz bezüglich der Rotation.
% TODO: Dieser Test funktioniert noch nicht richtig für PKM. Die Nullraum-
% bewegung für PKM scheitert an singulären Konfigurationen der Beinketten.
% 
% Siehe auch: cds_example_2T0R_taskred.m, cds_example_3T2R_taskred.m

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2021-10
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear

% Aufgaben-FG (Rotations-FG nicht belegt)
DoF = [1 1 1 0 0 0];

% Starte die Maßsynthese einmal mit Debug-Option für Redundanzkarte
for dbg_perfmap = [false, true] 
Traj_no = 1;
Set = cds_settings_defaults(struct('DoF', DoF));
Set.task.pointing_task = true;
Set.task.Ts = 1e-2;
Set.task.Tv = 1e-1;

Traj = cds_gen_traj(DoF, Traj_no, Set.task);

if dbg_perfmap
  Set.general.debug_task_redundancy = true; % Redundanzkarten erstellen
end
Set.optimization.objective = {'condition', 'chainlength'};
Set.optimization.constraint_obj(4) = 1000; % max. Wert für Konditionszahl
Set.optimization.NumIndividuals = 50;
Set.optimization.MaxIter = 20;
if dbg_perfmap
  Set.optimization.NumIndividuals = 3;
  Set.optimization.MaxIter = 4;
else
  Set.optimization.obj_limit = [1e3;1e3]; % Bei erstem Erfolg aufhören
end
Set.general.plot_details_in_fitness = 1e3;
Set.general.plot_robot_in_fitness = 1e3;
Set.general.verbosity = 3;
Set.general.matfile_verbosity = 3;
Set.structures.maxnumprismatic = 3; % für Portal-Systeme (bei seriellen)
Set.structures.min_task_redundancy = 1; % Fordere Aufgabenredundanz
Set.structures.max_task_redundancy = 1; % Erlaube Aufgabenredundanz
Set.general.eval_figures = {'robvisuanim'};
Set.general.save_robot_details_plot_fitness_file_extensions = {'png'};
Set.general.animation_styles = {'3D'};
Set.general.save_animation_file_extensions = {'mp4'};
Set.optimization.optname = '3T0sR_taskred_test1';
if dbg_perfmap
  Set.optimization.optname = [Set.optimization.optname, '_perfmap'];
end
Set.optimization.ee_translation = true; % Für SCARA benötigt, damit Aufgabenredundanz sinnvoll ist.
Set.structures.whitelist = {'S4RRRP1'};
cds_start(Set,Traj);

Set.optimization.optname = '3T0sR_taskred_test2';
if dbg_perfmap
  Set.optimization.optname = [Set.optimization.optname, '_perfmap2'];
end
% Beispielhafte 3T1R-PKM. TODO: Noch ungeklärtes Verhalten in IK. Unklar,
% ob PKM nicht dauerhaft singulär ist und gar nicht funktioniert.
Set.structures.whitelist = {'P4RRRRR5V1G1P1A1'};
Set.structures.use_serial = false;
Set.structures.max_index_active = 1;
Set.optimization.ee_translation = false;
Set.optimization.condition_limit_sing = inf; % Damit singuläre Beinketten nicht zum Abbruch führen
% Debug:
% Zum Debuggen auf dem Cluster rechnen lassen (alle möglichen Kinematiken)
% Set.general.computing_cluster = true;
% Set.general.computing_cluster_cores = 16;
% Set.general.cluster_maxrobotspernode = Set.general.computing_cluster_cores;
% if ~dbg_perfmap
%   Set.general.computing_cluster_max_time = 6*3600;
% else
%   Set.general.computing_cluster_max_time = 2*3600;
% end
% Set.general.create_template_functions = true;
cds_start(Set,Traj);
end