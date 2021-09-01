% Einstellungen für komb. Struktur und Maßsynthese für 2T1R Mechanismen
% Mehrkriterielle Optimierung mit verschiedenen Zielfunktionen.

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-08
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear

% Aufgaben-FG
DoF = [1 1 0 0 0 1];

Set = cds_settings_defaults(struct('DoF', DoF));
Set.task.Ts = 1e-2;
Set.task.Tv = 1e-1;
Set.optimization.NumIndividuals = 50;
Set.optimization.MaxIter = 30;
Set.general.plot_details_in_fitness = 0;
Set.general.plot_robot_in_fitness = 0;
Set.general.save_animation_file_extensions = {'gif'};
Set.general.maxduration_animation = 10;  % Länge begrenzen
Set.optimization.constraint_collisions = true;
Set.general.animation_styles = {'3D'};
Set.structures.whitelist = {'S3RRR1', 'P3RRR1G1P1A1'};
Traj = cds_gen_traj(DoF, 1, Set.task);
for obj_case = 1:5
  % Variablen zurücksetzen, die sonst als mit falscher Dimension gesetzt
  % erkannt werden.
  Set.optimization.obj_limit_physval = 0;
  Set.optimization.obj_limit = 0;
  switch obj_case
    case 1
      Set.optimization.objective = {'mass', 'condition'};
    case 2
      Set.optimization.objective = {'mass', 'energy'};
    case 3
      Set.optimization.objective = {'actforce', 'energy', 'condition'};
    case 4
      Set.optimization.objective = {'mass', 'energy', 'actforce', 'condition'};
    case 5
      Set.optimization.objective = {'jointrange', 'condition'};
  end
  Set.optimization.optname = sprintf('2T1R_motest_RRR_O%d', obj_case);
  cds_start(Set,Traj);
end
