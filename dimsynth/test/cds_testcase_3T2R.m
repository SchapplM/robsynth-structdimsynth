% Testskript für die Maßsynthese: 3T2R-Roboter mit verschiedenen
% Einstellungen. Durch dieses Skript sollen mögliche Fehlerfälle erkannt
% werden.

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-04
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear

DoF = [1 1 1 1 1 0]; % Aufgaben-FG
Set = cds_settings_defaults(struct('DoF', DoF));
Set.task.Ts = 1e-2;
Set.task.Tv = 1e-1;
Set.optimization.NumIndividuals = 30;
Set.optimization.MaxIter = 10;
% Set.general.plot_details_in_fitness = 1e3;
% Set.general.plot_robot_in_fitness = 1e3;
Set.general.save_robot_details_plot_fitness_file_extensions = {'fig'};
Set.general.save_animation_file_extensions = {'mp4'};
Set.general.matfile_verbosity = 1;
% Liste mit verschiedenen Beinketten und Koppelpunkten
whitelist_all = {'S5PRRRR8V2', 'S5RPRRR7V1', 'S5RRRRR10', ...
   'P5PRRRR4G1P8A1', 'P5PRRRR6G9P8A1', 'P5RRPRR4G2P8A1', 'P5RRRRR5G2P8A1'};
Traj = cds_gen_traj(DoF, 3, Set.task);
cds_show_task(Traj, Set);
for debugcalc = [0 1]
  for obj_name = {'condition', 'valid_act', 'mass', 'energy', 'actforce', 'materialstress', ...
      'stiffness', 'jointrange', 'manipulability', 'minjacsingval', 'positionerror', 'chainlength'}
    if strcmp(obj_name, 'valid_act') % nur für parallele Roboter
      Set.structures.whitelist = whitelist_all(~contains(whitelist_all, 'S'));
    else % für alle Roboter
      Set.structures.whitelist = whitelist_all;
    end
    if isempty(Set.structures.whitelist)
      continue % Kann beim Testen bei valid_act entstehen
    end
    Set.optimization.objective = obj_name;
    Set.general.debug_calc = logical(debugcalc);
    Set.optimization.optname = sprintf('testcase_3T2R_D-%d_O-%s', debugcalc, obj_name{1});
     % max. Wert für Konditionszahl (problematisch für 3T2R-PKM; wenn
     % Kondition fast singulär, dann sind die Ergebnisse nicht reproduzierbar)
    Set.optimization.constraint_obj(4) = 1000;
    cds_start
    % Ergebnisse laden und prüfen
    for j = 1:length(Structures)
      resmaindir = fullfile(Set.optimization.resdir, Set.optimization.optname);
      resdat = fullfile(resmaindir, sprintf('Rob%d_%s_Endergebnis.mat', j, Structures{j}.Name));
      if ~exist(resdat, 'file'), error('Ergebnisdatei %s nicht gefunden. Muss an dieser Stelle vorliegen', resdat); end
      tmp1 = load(resdat, 'RobotOptRes', 'Set', 'Traj');
      resdat2 = fullfile(resmaindir, sprintf('Rob%d_%s_Details.mat', j, Structures{j}.Name));
      tmp2 = load(resdat2, 'RobotOptDetails');
      clear cds_save_particle_details cds_fitness cds_log % notwendig, da Dimensionsänderung in persistenten Variablen
      fval_rtest = tmp2.RobotOptDetails.fitnessfcn(tmp1.RobotOptRes.p_val);
      abserr_fval = fval_rtest - tmp1.RobotOptRes.fval;
      relerr_fval = abserr_fval./tmp1.RobotOptRes.fval;
      if abs(abserr_fval) > 1e-4 && abs(relerr_fval) > 1e-2
        error('Fitness-Wert für %s nicht reproduzierbar', Structures{j}.Name);
      end
      if any(tmp1.RobotOptRes.fval > 1e8)
        % Bei einer 3T2R-PKM ist die Lösung kinematisch sehr ungünstig und
        % es wird aufgrund zu hoher Geschwindigkeit abgebrochen. Daher hier
        % nur Prüfung, ob die Trajektorie berechnet werden konnte
        error('Keine Lösung der Traj.-IK in Maßsynthese für %s gefunden. Fehler?', Structures{j}.Name);
      end
    end
  end
end
