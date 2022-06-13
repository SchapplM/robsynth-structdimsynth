% Beispiel für Maßsynthese mit Entwurfsoptimierung und Aufgabenredundanz
% Szenario: 3RRR-PKM, die an der Wand befestigt ist. Kompensationsfedern
% gleichen die Gravitation aus. Die Federn werden optimiert.

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2022-06
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear

%% Einstellungen für Aufgabe
% Debug: Ohne Redundanz prüfen
nonredundant = false;

DoF = [1 1 0 0 0 0]; % Aufgaben-FG
if nonredundant
  DoF(end) = 1;
end

Traj_no = 1;
Set = cds_settings_defaults(struct('DoF', DoF));
Set.task.pointing_task = true; % notwendig, damit Aufgabenredundanz für 2T0R funktioniert.
% Debug: Gröbere Abtastung. Führt teilweise zu Korrelationsfehlern in Traj-IK
% Set.task.Ts = 1e-2; % gröbere Abtastung
% Set.task.Tv = 1e-1;
Set.task.wall_rotate_traj = true;
Traj = cds_gen_traj(DoF, Traj_no, Set.task);

% Wandmontage, damit der g-Vektor in der Bewegungsebene des Roboters ist
Set.structures.mounting_parallel = 'wall';
% Keine Selbstkollisionen (sieht sonst unplausibel im Bild aus)
Set.optimization.constraint_collisions = true;
Set.structures.max_task_redundancy = 1; % Erlaube Aufgabenredundanz
if nonredundant
  Set.structures.max_task_redundancy = 0;
end

%% Einstellungen für Optimierung und Ergebnis-Formatierung
Set.optimization.objective = {'actforce', 'mass'};
Set.optimization.NumIndividuals = 20;
Set.optimization.MaxIter = 5;
Set.optimization.optname = '2T0R_taskred_desopt_test_3RRR';
Set.structures.whitelist = {'P3RRR1G1P1A1'};
% Debug: Bei erstem Erfolg aufhören. Hat auch Auswirkung auf die Entwurfsopt.
Set.optimization.obj_limit = [1e3;1e3];

% Debug:
% Set.general.plot_details_in_fitness = 4e9; % Bei Selbstkollision (Eckpunkt) plotten
% Set.general.plot_robot_in_fitness = 1e3;
Set.general.verbosity = 3;
Set.general.matfile_verbosity = 2;

Set.general.save_robot_details_plot_fitness_file_extensions = {'png'};
Set.general.update_template_functions = false; % geht schneller
Set.general.animation_styles = {'3D'};
Set.general.save_animation_file_extensions = {'mp4'};
Set.general.eval_figures = {'robvisuanim', 'pareto_dimsynth_desopt', 'pareto_all_phys'};

%% Einstellungen für Entwurfsoptimierung
% Optimiere die Steifigkeit und Nullstellung der Gelenkelastizitäten als
% Entwurfsopt. (Noch nicht implementiert)
Set.optimization.desopt_vars = {'joint_stiffness_qref', 'joint_stiffness'};
Set.optimization.joint_stiffness_passive_revolute = NaN;
Set.optimization.joint_stiffness_passive_universal = NaN;
Set.optimization.joint_stiffness_active_revolute = 0;
% Debug:
% Set.general.plot_details_in_desopt = 1e3;
Set.optimization.desopt_use_obj_limit = false; % Debug: Falls nur eine Iteration in Maßsynthese gewünscht.
Set.optimization.desopt_MaxIter = 20; % mehr als Standard
Set.optimization.desopt_NumIndividuals = 40;
Set.general.debug_desopt = true;

% Debug: Nur statische Kräfte und keine Masse der Beinketten annehmen.
% Auch keine Betrachtung der Plattform-Masse jenseits der definierten
% Traglast. Dadurch keine Betrachtung von Beschleunigungskräften und rein
% statische Auslegung.
% Set.optimization.static_force_only = true;
% Set.optimization.nolinkmass = true;
% Set.optimization.noplatformmass = true;

% Debug: Nur Plotten der Ergebnisse (wenn Daten schon gespeichert sind)
% Set.general.regenerate_summary_only = true;
% Set.general.only_finish_aborted = true;
cds_start(Set,Traj);
