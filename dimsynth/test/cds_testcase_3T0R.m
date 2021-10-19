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
Set.general.matfile_verbosity = 1;
Set.general.plot_details_in_fitness = 1e3;
Set.general.plot_robot_in_fitness = 1e3;
Set.general.save_robot_details_plot_fitness_file_extensions = {'fig'};
Set.general.save_animation_file_extensions = {'mp4', 'gif'};
% Liste mit verschiedenen Beinketten und Koppelpunkten
whitelist_all = {'P3PRRRR1G1P3A1', 'P3RPRRR8G4P2A1', 'P3RRRRR1G3P3A2'};
Traj = cds_gen_traj(DoF, 1, Set.task);
for debugcalc = [0 1]
  for obj_name = {'valid_act', 'mass', 'energy', 'condition', 'actforce', ...
      'materialstress', 'stiffness', 'jointrange', 'manipulability', ...
      'minjacsingval', 'positionerror', 'actvelo', 'chainlength', 'installspace', 'footprint'}
    if strcmp(obj_name, 'valid_act') % nur für parallele Roboter
      Set.structures.whitelist = whitelist_all(~contains(whitelist_all, 'S'));
    else % für alle Roboter
      Set.structures.whitelist = whitelist_all;
    end
    Set.optimization.objective = obj_name;
    Set.general.debug_calc = logical(debugcalc);
    Set.optimization.optname = sprintf('testcase_3T0R_D-%d_O-%s', debugcalc, obj_name{1});
    cds_start(Set,Traj);
    % Ergebnisse laden und prüfen
    resmaindir = fullfile(Set.optimization.resdir, Set.optimization.optname);
    ds = load(fullfile(resmaindir, [Set.optimization.optname, '_settings.mat']));
    Structures = ds.Structures;
    for j = 1:length(Structures)
      resdat1 = fullfile(resmaindir, sprintf('Rob%d_%s_Endergebnis.mat', j, Structures{j}.Name));
      if ~exist(resdat1, 'file'), error('Ergebnisdatei %s nicht gefunden. Muss an dieser Stelle vorliegen', resdat1); end
      tmp1 = load(resdat1, 'RobotOptRes');
      resdat2 = fullfile(resmaindir, sprintf('Rob%d_%s_Details.mat', j, Structures{j}.Name));
      tmp2 = load(resdat2, 'RobotOptDetails');
      clear cds_save_particle_details cds_fitness cds_log % notwendig, da Dimensionsänderung in persistenten Variablen
      fval_rtest = tmp2.RobotOptDetails.fitnessfcn(tmp1.RobotOptRes.p_val);
      abserr_fval = fval_rtest - tmp1.RobotOptRes.fval;
      relerr_fval = abserr_fval./tmp1.RobotOptRes.fval;
      if abs(abserr_fval) > 1e-4 && abs(relerr_fval) > 1e-2
        error('Fitness-Wert für %s nicht reproduzierbar', Structures{j}.Name);
      end
      if any(tmp1.RobotOptRes.fval > 1e3)
        error('Keine Lösung in Maßsynthese für %s gefunden. Fehler?', Structures{j}.Name);
      end
    end
  end
end
