% Einstellungen für Test der komb. Synthese für 2T0R-Aufgabenredundanz

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2021-02
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear

DoF = [1 1 0 0 0 0]; % Aufgaben-FG

% Starte die Maßsynthese einmal mit Debug-Option für Redundanzkarte
for dbg_perfmap = [false, true] 
Traj_no = 1;
Set = cds_settings_defaults(struct('DoF', DoF));
Set.task.pointing_task = true; % notwendig, damit Aufgabenredundanz für 2T0R funktioniert.
Set.task.Ts = 1e-2; % gröbere Abtastung
Set.task.Tv = 1e-1;
Traj = cds_gen_traj(DoF, Traj_no, Set.task);

if dbg_perfmap
  Set.general.debug_task_redundancy = true; % Redundanzkarten erstellen
end
Set.optimization.objective = {'condition', 'chainlength'};
Set.optimization.NumIndividuals = 50;
Set.optimization.MaxIter = 20;
if dbg_perfmap
  Set.optimization.NumIndividuals = 3;
  Set.optimization.MaxIter = 4;
else
  Set.optimization.obj_limit = [1e3;1e3]; % Bei erstem Erfolg aufhören
end
Set.general.plot_details_in_fitness = 0;
Set.general.plot_robot_in_fitness = 0;
Set.general.verbosity = 3;
Set.general.matfile_verbosity = 3;
Set.structures.use_parallel_rankdef = 6; % für 2PP-PKM notwendig.
Set.structures.nopassiveprismatic = false;
Set.structures.max_task_redundancy = 1; % Erlaube Aufgabenredundanz
Set.general.eval_figures = {};
Set.general.save_robot_details_plot_fitness_file_extensions = {'png'};
Set.general.animation_styles = {};
Set.general.eval_figures = {'robvisuanim', 'pareto_all_phys', 'pareto'};
% Zwei Optimierungen starten, da manche Roboter einen zusätzliche EE-Hebel
% brauchen (sonst letztes Gelenk direkt der Endeffektor).
Set.optimization.optname = '2T0R_taskred_test1';
if dbg_perfmap
  Set.optimization.optname = [Set.optimization.optname, '_perfmap'];
end
Set.structures.whitelist = {'S3RRR1', 'S2RR3'};
Set.optimization.ee_translation = true; % Für S2RR3 benötigt. Für S3RRR1 sinnvoll.
cds_start

Set.optimization.optname = '2T0R_taskred_test2';
if dbg_perfmap
  Set.optimization.optname = [Set.optimization.optname, '_perfmap'];
end
Set.structures.whitelist = {'S2PP1', 'P2PP1G1P1A1', 'P3RPR1G1P1A1'};
Set.optimization.ee_translation = false; % Hier nicht benötigt.
cds_start

end
