% Prüfe, ob die PKM-Modelle mit G1 und G4 Gestell-Ausrichtung überführbar
% sind. Für den Sonderfall der senkrechten Anordnung sind sie gleich.
% Ergebnis:
% * Exakt gleiches Ergebnis. Kein Fehler. Modelle sind überführbar.

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-09
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clear
clc
offline = false;
tmpdir = fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp');
for iFG = 1%1:3
  %% Vorbereiten
  if iFG == 1
    DoF = [1 1 1 0 0 0];
    % erst die gerade Anordnung, dann die schräge
    whitelist = {'P3PRRRR8V1G1P2A1', 'P3PRRRR8V1G4P2A1'};
  elseif iFG == 2
    DoF = [1 1 1 0 0 1]; % TODO: Die PKM sind noch nicht ganz passend
    whitelist = {'P4RPRRR5G1P1A1', 'P4RPRRR5G4P1A1'};
  elseif iFG == 3
    DoF = [1 1 1 1 1 1]; % TODO: Die PKM sind noch nicht ganz passend
    whitelist = {'P6PRRRRR6G7P5A1', 'P6PRRRRR6G8P5A1'};
  else
    error('Fall nicht definiert');
  end
  Set = cds_settings_defaults(struct('DoF', DoF));
  Set.optimization.NumIndividuals = 30;
  Set.optimization.MaxIter = 2;
  Set.optimization.base_morphology = true; % Sonst wird der Unterschied G1/G4 nicht sichtbar
  % Beide Zielfunktionen benutzen. Damit gehen Grenzen für beide als Abbruchbedingung.
  Set.optimization.objective = {'condition','mass'};
  Set.structures.whitelist = whitelist;
  Set.optimization.optname = sprintf('testcase_G1G4_%dT%dR_dbg2', sum(DoF(1:3)), sum(DoF(4:6)));
  Set.optimization.obj_limit_physval = [0.1; 0.3]; % [50;200]; % Nehme erstes Ergebnis mit brauchbarer Konditionszahl und moderater Antriebskraft
  Set.general.create_template_functions = true;
  Set.general.nosummary = true;
  Set.general.matfile_verbosity = 3;
  Traj = cds_gen_traj(DoF, 1, Set.task);
  %% Starten
  if ~offline
    cds_start
  else % Direkte Auswertung der vorher generierten Ergebnisse
    parroblib_addtopath(whitelist);
  end

  %% Auswertung: Überführung G1-G4
  fprintf('Optimierung abgeschlossen. Reproduziere Ergebnisse\n');
  % Ergebnisse erneut laden
  resmaindir = fullfile(Set.optimization.resdir, Set.optimization.optname);
  DG1 = load(fullfile(resmaindir, ...
    sprintf('Rob%d_%s_Endergebnis.mat', 1, whitelist{1})), 'RobotOptRes', 'Set', 'Traj');
  DG4 = load(fullfile(resmaindir, ...
    sprintf('Rob%d_%s_Endergebnis.mat', 2, whitelist{2})), 'RobotOptRes', 'Set', 'Traj');
  if any(DG1.RobotOptRes.fval > 1e3) || any(DG4.RobotOptRes.fval > 1e3)
    error('Keine Lösung in Maßsynthese gefunden. Das hat sonst funktionert. Fehler?');
  end
  % Reproduziere die Ergebnisse
  clear cds_save_particle_details cds_fitness cds_log % notwendig, da Dimensionsänderung in persistenten Variablen
  fval_G1_rtest = DG1.RobotOptRes.fitnessfcn(DG1.RobotOptRes.p_val);
  if any(abs(fval_G1_rtest - DG1.RobotOptRes.fval) > 1e-6)
    error('Fitness-Wert für G1 nicht reproduzierbar');
  end
  clear cds_save_particle_details cds_fitness cds_log
  fval_G4_rtest = DG4.RobotOptRes.fitnessfcn(DG4.RobotOptRes.p_val);
  if any(abs(fval_G4_rtest - DG4.RobotOptRes.fval) > 1e-6)
    error('Fitness-Wert für G4 nicht reproduzierbar');
  end
  fprintf('Validiere Ergebnisse zwischen G1 und G4 (Parametertransfer)\n');
  % Setze Parameter von G1 in G4 ein
  p_G1 = DG1.RobotOptRes.p_val;
  p_G4 = NaN(size(DG4.RobotOptRes.p_val));
  for j = 1:length(p_G4)
    I_G1G4j = strcmp(DG1.RobotOptRes.Structure.varnames, DG4.RobotOptRes.Structure.varnames{j});
    if sum(I_G1G4j) ~= 1, continue; end
    p_G4(j) = p_G1(I_G1G4j);
  end
  Ip_G4elev = strcmp(DG4.RobotOptRes.Structure.varnames, 'base_morph_coneelev');
  if sum(Ip_G4elev) == 0, error('Steigungsparameter kommt nicht vor.'); end
  p_G4(Ip_G4elev) = 0; % Winkel 0 heißt senkrecht nach oben, wie G1.
  % Berechne die Fitness-Funktionen der gleichwertigen Parameter
  clear cds_save_particle_details cds_fitness cds_log
  [fval_G1_test, Q_G1_test] = DG1.RobotOptRes.fitnessfcn(p_G1);
  clear cds_save_particle_details cds_fitness cds_log
  [fval_G4_test, Q_G4_test] = DG4.RobotOptRes.fitnessfcn(p_G4);
  test_G1G4 = fval_G1_test - fval_G4_test;
  if any(abs(test_G1G4) > 1e-8)
    error('PKM mit G4-Modell stimmt nicht mit G1-Modell überein (%s vs %s). Fehler.', ...
      whitelist{1}, whitelist{2});
  end
  % Teste nochmal mit senkrecht nach unten. Das Ergebnis sollte eigentlich
  % gleich sein?
  p_G4_pi = p_G4;
  p_G4_pi(Ip_G4elev) = pi; % ist exakt entgegengesetzt zu G1.
  clear cds_save_particle_details cds_fitness cds_log
  [fval_G4_test2, Q_G4_test2] = DG4.RobotOptRes.fitnessfcn(p_G4_pi);
  test_G4pi = fval_G4_test2 - fval_G4_test;
  R = DG4.RobotOptRes.R;
  if any(abs(test_G4pi) > 1e-4)
    error('PKM mit G4-Modell hat anderes Ergebnis, wenn Schubachse umgedreht ist. Unlogisch.');
    % Vergleich der Gelenkwinkelverläufe
    figure(2);clf; RP = ['R', 'P']; %#ok<UNRCH>
    for i = 1:R.NJ
      legnum = find(i>=R.I1J_LEG, 1, 'last');
      legjointnum = i-(R.I1J_LEG(legnum)-1);
      subplot(ceil(sqrt(R.NJ)), ceil(R.NJ/ceil(sqrt(R.NJ))), i);
      hold on; grid on;
      plot(Q_G1_test(:,i), '-');
      plot(Q_G4_test(:,i), '--');
      plot(Q_G4_test2(:,i), '-');
      title(sprintf('q %d (%s), L%d,J%d', i, RP(R.MDH.sigma(i)+1), legnum, legjointnum));
    end
    legend({'G1', 'G4,alpha=0', 'G4,alpha=pi'});
    % Vergleich der Roboter-Bilder
    Set = DG4.Set;
    Set.general.plot_robot_in_fitness = 1e3;
    ff = @(p)cds_fitness(DG4.RobotOptRes.R,Set,DG4.Traj,DG4.RobotOptRes.Structure,p(:));
    clear cds_save_particle_details cds_fitness cds_log
    ff(p_G4);
    saveas(200, fullfile(tmpdir, 'testcase_G1G4_Rob_alpha_null.fig')); close(200);
    clear cds_save_particle_details cds_fitness cds_log
    ff(p_G4_pi);
    saveas(200, fullfile(tmpdir, 'testcase_G1G4_Rob_alpha_pi.fig')); close(200);
    uiopen(fullfile(tmpdir, 'testcase_G1G4_Rob_alpha_null.fig'), true);
    uiopen(fullfile(tmpdir, 'testcase_G1G4_Rob_alpha_pi.fig'), true);
  end
end
fprintf('PKM mit G1 und G4 gegeneinander getestet. Identisches Ergebnis. Alles i.O.\n');