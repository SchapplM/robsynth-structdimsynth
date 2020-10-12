% Testskript für die Maßsynthese: 3T3R-Roboter mit verschiedenen
% Einstellungen. Durch dieses Skript sollen mögliche Fehlerfälle erkannt
% werden.

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-04
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover


% Aufgaben-FG
DoF = [1 1 1 1 1 1];

Set = cds_settings_defaults(struct('DoF', DoF));
Set.task.Ts = 1e-2;
Set.task.Tv = 1e-1;
Set.optimization.NumIndividuals = 30;
Set.optimization.MaxIter = 10;
Set.general.plot_details_in_fitness = 1e3;
Set.general.plot_robot_in_fitness = 1e3;
Set.general.save_robot_details_plot_fitness_file_extensions = {'fig'};
Set.general.save_animation_file_extensions = {'mp4', 'gif'};
Set.general.matfile_verbosity = 1;
% Liste mit verschiedenen Beinketten und Koppelpunkten
whitelist_all = {'S6RRRRRR10V2', 'S6RRRRRR10', 'P6PRRRRR6G8P1A1', ...
  'P6PRRRRR6V2G8P4A1', 'P6RRRRRR10V3G1P1A1', 'P6RRRRRR10G1P1A1'};
Traj = cds_gen_traj(DoF, 1, Set.task);
for debugcalc = [0 1]
  %% Optimiere jedes mögliche Zielkriterium einzeln
  obj_list = {'valid_act', 'mass', 'energy', 'condition', 'actforce', ...
    'stiffness', 'jointrange'};
  % Speichere Erfolg aller Maßsynthese-Versuche ab. Auswertung unten.
  success_matrix = false(length(whitelist_all), length(obj_list));
  for obj_name = obj_list
    if strcmp(obj_name, 'valid_act') % nur für parallele Roboter
      Set.structures.whitelist = whitelist_all(~contains(whitelist_all, 'S'));
    else % für alle Roboter
      Set.structures.whitelist = whitelist_all;
    end
    Set.optimization.objective = obj_name;
    Set.general.debug_calc = logical(debugcalc);
    Set.optimization.optname = sprintf('testcase_3T3R_D-%d_O-%s', debugcalc, obj_name{1});
    cds_start
    % Ergebnisse laden und prüfen
    for j = 1:length(Structures)
      resmaindir = fullfile(Set.optimization.resdir, Set.optimization.optname);
      resdat = fullfile(resmaindir, sprintf('Rob%d_%s_Endergebnis.mat', j, Structures{j}.Name));
      if ~exist(resdat, 'file'), error('Ergebnisdatei %s nicht gefunden. Muss an dieser Stelle vorliegen', resdat); end
      tmp=load(resdat, 'RobotOptRes', 'Set', 'Traj');
      clear cds_save_particle_details cds_fitness cds_log % notwendig, da Dimensionsänderung in persistenten Variablen
      fval_rtest = tmp.RobotOptRes.fitnessfcn(tmp.RobotOptRes.p_val);
      abserr_fval = fval_rtest - tmp.RobotOptRes.fval;
      relerr_fval = abserr_fval./tmp.RobotOptRes.fval;
      if abs(abserr_fval) > 1e-4 && abs(relerr_fval) > 1e-2
        error('Fitness-Wert für %s nicht reproduzierbar', Structures{j}.Name);
      end
      if any(tmp.RobotOptRes.fval > 1e3)
        warning('Keine Lösung in Maßsynthese für %s gefunden. Muss kein Fehler sein.', Structures{j}.Name);
      else
        success_matrix(strcmp(Structures{j}.Name,whitelist_all), ...
          strcmp(obj_name,obj_list)) = true;
      end
    end
  end
  %% Prüfe, ob für einen Roboter nie eine Lösung gefunden wurde. 
  % Annahme: Bei so vielen Zielkriterien muss durch Zufall für jeden einmal eine
  % gültige Lösung gefunden werden. Ansonsten ist die Auswahl oben schlecht.
  for j = 1:length(whitelist_all)
    if ~any(success_matrix(j,:))
      error('Für Rob %d (%s) wurde nie eine gültige Lösung gefunden.', j, whitelist_all{j});
    elseif ~all(success_matrix(j,:))
      warning('Für Rob %d (%s) führten nur %d/%d Optimierungen zu einer gültigen Lösung. Nicht gültig:', ...
        j, whitelist_all{j}, sum(success_matrix(j,:)), size(success_matrix,2));
      disp(obj_list(~success_matrix(j,:)));
    else
      fprintf('Maßsynthese für Rob %d (%s) %d mal erfolgreich\n', ...
        j, whitelist_all{j}, sum(success_matrix(j,:)));
    end
  end
end
