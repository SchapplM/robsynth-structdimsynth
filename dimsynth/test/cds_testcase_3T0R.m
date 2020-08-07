% Testskript für die Maßsynthese: 3T0R-Roboter mit verschiedenen
% Einstellungen. Durch dieses Skript sollen mögliche Fehlerfälle erkannt
% werden.

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-04
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover


% Aufgaben-FG
DoF = [1 1 1 0 0 0];

Set = cds_settings_defaults(struct('DoF', DoF));
Set.task.Ts = 1e-2;
Set.task.Tv = 1e-1;
Set.optimization.NumIndividuals = 30;
Set.optimization.MaxIter = 10;
Set.general.plot_details_in_fitness = 1e3;
Set.general.plot_robot_in_fitness = 1e3;
Set.general.save_robot_details_plot_fitness_file_extensions = {'fig'};
Set.general.save_animation_file_extensions = {'mp4', 'gif'};
% Liste mit verschiedenen Beinketten und Koppelpunkten
whitelist_all = {'P3PRRRR1G1P3A1', 'P3PRRRR8G1P1A1', 'P3PRRRR4G2P2A1', ...
  'P3RRRRR1G3P3A2', 'P3RPRRR8G4P2A1', 'P3RRRRR10G2P2A1'};
Traj = cds_gen_traj(DoF, 1, Set.task);
for debugcalc = [0 1]
  for obj_name = {'valid_act', 'mass', 'energy', 'condition', 'actforce', 'stiffness'}
    if strcmp(obj_name, 'valid_act') % nur für parallele Roboter
      Set.structures.whitelist = whitelist_all(~contains(whitelist_all, 'S'));
    else % für alle Roboter
      Set.structures.whitelist = whitelist_all;
    end
    Set.optimization.objective = obj_name;
    Set.general.debug_calc = logical(debugcalc);
    Set.optimization.optname = sprintf('testcase_3T0R_D-%d_O-%s', debugcalc, obj_name{1});
    cds_start
  end
end
