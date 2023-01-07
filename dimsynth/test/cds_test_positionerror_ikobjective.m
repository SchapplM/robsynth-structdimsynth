% Teste, ob das Positionsfehler-Kriterium in der Redundanzauflösung und in
% der Maßsynthese konsistent sind.
% 
% Ablauf:
% * Maßsynthese für verschiedene PKM berechnen
% * Redundanzkarte für erfolgreiche Lösung berechnen (mit Positionsfehler)
% * Positionsfehler-Kriterium separat berechnen
% * Vergleichen, ob die Werte gleich sind

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2023-01
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear

% Aufgaben-FG
for iDoF = 1:3
  switch iDoF
    % Wähle verschiedene Strukturen: 2T1R, 3T1R, 3T3R, Seriell/Parallel
    case 1
      DoF = [1 1 0 0 0 0];
      whitelist = {'S3RRR1', 'P3RPR1G1P1A2'};
    case 2
      DoF = [1 1 1 0 0 0];
      whitelist = {'S4RRRP1', 'P4RRRRR8G1P1A4'};
    case 3
      DoF = [1 1 1 1 1 0];
      whitelist = {'S6RRRRRR10V2', 'P6PRRRRR6V4G8P1A1'};
  end
  Traj_no = 1;
  
  Set = cds_settings_defaults(struct('DoF', DoF));
  Set.task.pointing_task = true; % notwendig, damit Aufgabenredundanz für 2T0R funktioniert.
  Set.task.Ts = 1e-2;
  Set.task.Tv = 1e-1;
  
  % Schwenkwinkel etwas verringern (ist sonst zu schwer erreichbar und das
  % hier ist ja nur ein Testskript)
  Set.task.maxangle = 30*pi/180;
  Traj = cds_gen_traj(DoF, Traj_no, Set.task);
  
  %% Weitere Einstellungen
  Set.optimization.objective = {'positionerror'};
  Set.optimization.objective_ik = 'poserr_ee';
  % Der Positionsfehler muss gering sein (mit Standard-Encoder-Genauigkeit
  % möglich). Sonst vermutlich Singularität
  Set.optimization.constraint_obj(7) = 0.5e-3; % 500µm
  % Wähle mittelhohen Grenzwert für Konditionszahl um keine Singularitäten
  % in Ergebnissen zu haben
  Set.optimization.constraint_obj(4) = 5e3;
  Set.optimization.optname = sprintf('positionerror_ikobj_test_%dT%dR', ...
    sum(DoF(1:3)), sum(DoF(4:6)));
  Set.optimization.NumIndividuals = 50;
  Set.optimization.MaxIter = 20;
  Set.optimization.obj_limit = 1e3; % Bei erstem Erfolg aufhören
  Set.general.taskred_dynprog = false;
  % Set.optimization.max_range_passive_spherical = 240*pi/180; % Technisch unrealistisch, aber für Ergebnis-Veranschaulichung besser.
  Set.general.plot_robot_in_fitness = 0;%1e3; % Bei Erfolg
  Set.general.plot_details_in_fitness = 0;%4e9; % Debug-Plots für Selbstkollision
  Set.general.verbosity = 3;
  Set.general.matfile_verbosity = 3;
  Set.optimization.ee_rotation = true;
  Set.optimization.ee_translation = false; % für S3RRR1 evtl. sinnvoll. Sonst AR ohne Wirkung
  Set.structures.max_task_redundancy = 1;
  Set.general.eval_figures = {};
  Set.general.animation_styles = {};
  Set.general.debug_calc = false;
  Set.optimization.traj_ik_abort_on_success = true; % Sofort aufhören
  % Auswertungsbild zum Debuggen der Aufgabenredundanz
  % Set.general.debug_taskred_perfmap = true;
  Set.general.save_robot_details_plot_fitness_file_extensions = {'png', 'fig'};
  Set.structures.whitelist = whitelist;
  cds_start(Set,Traj);
  
  %% Ergebnisse laden und prüfen
  resmaindir = fullfile(Set.optimization.resdir, Set.optimization.optname);
  ds = load(fullfile(resmaindir, [Set.optimization.optname, '_settings.mat']));
  Structures = ds.Structures;
  for j = 1:length(Structures)
    fprintf('Rob. %d: Prüfe Konsistenz der Funktionen für %s\n', j, Structures{j}.Name);
    resdat1 = fullfile(resmaindir, sprintf('Rob%d_%s_Endergebnis.mat', j, Structures{j}.Name));
    if ~exist(resdat1, 'file'), error('Ergebnisdatei %s nicht gefunden. Muss an dieser Stelle vorliegen', resdat1); end
    tmp1 = load(resdat1, 'RobotOptRes');
    resdat2 = fullfile(resmaindir, sprintf('Rob%d_%s_Details.mat', j, Structures{j}.Name));
    tmp2 = load(resdat2, 'RobotOptDetails');
    R = tmp2.RobotOptDetails.R;
    cds_save_particle_details(); cds_fitness(); cds_log(); % notwendig, da Dimensionsänderung in persistenten Variablen
    Structure_tmp = tmp1.RobotOptRes.Structure;
    Structure_tmp.q0_traj = tmp1.RobotOptRes.q0;
    Set_tmp = Set;
    Set_tmp.optimization.pos_ik_abort_on_success = true;
    Set_tmp.optimization.traj_ik_abort_on_success = true; % Sofort aufhören
    Set_tmp.general.debug_taskred_perfmap = 1; % Redundanzkarte erzeugen
    Set_tmp.general.save_robot_details_plot_fitness_file_extensions = {'fig', 'png'};
    fprintf('Erneute Berechnung der Fitness-Funktion (um Redundanzkarte zu generieren)\n');
    [fval_rtest2, ~, Q, ~, ~, ~, ~, Jinv] = cds_fitness(R, Set_tmp, Traj, ...
      Structure_tmp, tmp1.RobotOptRes.p_val, tmp1.RobotOptRes.desopt_pval);
    Traj_0 = cds_transform_traj(R, Traj);
    % Redundanzkarte laden
    tmpdir_j = fullfile(resmaindir, 'tmp', sprintf('%d_%s', ...
      Structure_tmp.Number, Structure_tmp.Name));
    assert(exist(tmpdir_j, 'file'), 'Tmp-Verzeichnis existiert nicht');
    trajdatafiles = dir(fullfile(tmpdir_j, '*_Traj*.mat'));
    perfmapfiles = dir(fullfile(tmpdir_j, '*Konfig*TaskRedPerfMap_Data.mat'));
    assert(length(perfmapfiles)==1, sprintf('Unerwartete Anzahl an Redundanzkarten-Daten (%d)', length(perfmapfiles)));
    dpm = load(fullfile(tmpdir_j, perfmapfiles(1).name));
    % Generiere Redundanzkarte nochmal neu, damit auch die Gelenkwinkel aus- 
    % gegeben werden (wird normalerweise nicht gemacht wegen Speicherplatz)
    fprintf('Neugenerierung der Redundanzkarte\n');
    dpm.set_perfmap.verbose = true;
    [H_all, Q_all, s_ref, s_tref, phiz_range] = R.perfmap_taskred_ik( ...
      Traj_0.X, Traj_0.IE(Traj_0.IE~=0), dpm.set_perfmap);
    % Ordne jedem Schritt der Redundanzkarte einen Index der Trajektorie zu
    I_t = NaN(length(s_ref),1);
    for ii = 1:length(s_ref)
      % Suche das Vorkommnis mit Differenz und nicht mit Gleichheit
      [s_error, I_tii] = min(abs(s_tref - s_ref(ii)));
      assert(length(I_tii)==1, 'Problem bei Zuordnung');
      I_t(ii) = I_tii; % Alternativ: find(s_tref == s_ref(ii));
    end
    %% Gehe alle Punkte durch und vergleiche die Kriterien
    Hpos_pm = H_all(:,:,R.idx_ikpos_hn.poserr_ee);
    Hcond = H_all(:,:,end);
    I_sing = Hcond > 1e6; % Diese Einträge werden nicht untersucht. Abweichung numerisch zu groß und zu anfällig gegen kleine Abweichungen.
    fprintf('Neue Redundanzkarte: %d / %d (%d x %d) Einträge ungleich NaN. %d Einträge singulär.\n', ...
      sum(~isnan(Hpos_pm(:))), numel(Hpos_pm), size(Hpos_pm, 1), size(Hpos_pm, 2), sum(I_sing(:)));
    Hpos_err = NaN(size(H_all,1), size(H_all,2)); Hpos_fcn = Hpos_err;
    num_checks = 0;
    for ii = 1:size(H_all,1)
      for jj = 1:size(H_all,2)
        if any(isnan(Q_all(jj,:,ii))), continue; end
        % Positionsfehler aus Redundanzkarte auslesen
        hpe_perfmap = H_all(ii,jj,R.idx_ikpos_hn.poserr_ee);
        % Zum Eintrag der Redundanzkarte passende EE-Pose (u.a. zum Testen)
        xE = [Traj_0.X(I_t(ii),1:5)'; phiz_range(jj)];
        xE_fromq = R.fkineEE_traj(Q_all(jj,:,ii))';
        assert(all(abs(xE(1:3)-xE_fromq(1:3)) < 1e-4), ...
          'Gelenkwinkel passen nicht zu gewähltem x (translatorisch)');
        assert(all(wrapToPi(abs(xE(4:6)-xE_fromq(4:6))) < 1e-4), ...
          'Gelenkwinkel passen nicht zu gewähltem x (rotatorisch)');
        % Positionsfehler mit Maßsynthese-Zielfunktion berechnen
        if R.Type == 0 % Seriell
          Jinvges = [];
        else % PKM
          [~,Jinvges] = R.jacobi_qa_x(Q_all(jj,:,ii)', xE);
        end
        [~,~,~,hpe_function] = cds_obj_positionerror(R, Set_tmp, Jinvges(:)', Q_all(jj,:,ii));
        num_checks = num_checks + 1;
        Hpos_fcn(ii,jj) = hpe_function;
        Hpos_err(ii,jj) = hpe_perfmap-hpe_function;
      end % for jj
    end % for ii
    assert(num_checks ~= 0, 'Keine Prüfung durchgeführt. Logik-Fehler.');
    Hpos_err_rel = Hpos_err ./ Hpos_pm;
    I_err = abs(Hpos_err_rel) > 5e-2 & abs(Hpos_err) > 1e-6 & ~I_sing;
    if any(I_err(:))
      % Zeichne die Redundanzkarten mit Positionsfehlern aus beiden
      % Methoden.
      wn_pmp = zeros(R.idx_ik_length.wnpos,1);
      wn_pmp(R.idx_ikpos_wn.poserr_ee) = 1;
      H_all_pmp = H_all;
      H_all_pmp(:,:,R.idx_ikpos_hn.poserr_ee) = Hpos_pm;
      settings_perfmapplot = struct('wn', wn_pmp, 'TrajLegendText', {{}}, ...
        'i_ar', 0, 'name_prefix_ardbg', '', 'fval', 0, 'logscale', true, ...
        'critnames', {fields(R.idx_ikpos_wn)'}, 'constrvioltext', '');
      pmfig_hpm = cds_debug_taskred_perfmap(Set_tmp, Structures{j}, H_all_pmp, s_ref, ...
        s_tref, phiz_range, NaN(length(s_tref),0), NaN(length(s_tref),0), settings_perfmapplot);
      set(pmfig_hpm, 'Name', 'PerfMap_from_ik', 'NumberTitle', 'off');
      sgtitle('Position Error from IK (PerfMap function)');
      H_all_pmp(:,:,R.idx_ikpos_hn.poserr_ee) = Hpos_fcn;
      settings_perfmapplot.i_ar = 1; % Damit andere Bildnummer benutzt wird
      pmfig_fcn = cds_debug_taskred_perfmap(Set_tmp, Structures{j}, H_all_pmp, s_ref, ...
        s_tref, phiz_range, NaN(length(s_tref),0), NaN(length(s_tref),0), settings_perfmapplot);
      set(pmfig_fcn, 'Name', 'PerfMap_from_cds', 'NumberTitle', 'off');
      sgtitle('Position Error from DimSynth Function');
      H_all_pmp(:,:,R.idx_ikpos_hn.poserr_ee) = Hpos_err;
      settings_perfmapplot.i_ar = 2;
      pmfig_err = cds_debug_taskred_perfmap(Set_tmp, Structures{j}, H_all_pmp, s_ref, ...
        s_tref, phiz_range, NaN(length(s_tref),0), NaN(length(s_tref),0), settings_perfmapplot);
      set(pmfig_err, 'Name', 'PerfMap_error', 'NumberTitle', 'off');
      sgtitle('Deviation of Position Error by both Methods');
      error('Abweichung zwischen Positionsfehler aus Redundanzkarte und Maßsynthese-Zielfunktion');
    else
      fprintf(['Rob. %d (%s): Übereinstimmung zwischen Ergebnis aus ', ...
        'Redundanzkarte und Maßsynthese-Zielfunktion\n'], j, Structures{j}.Name);
    end
  end % for Structures
end % for iDoF
